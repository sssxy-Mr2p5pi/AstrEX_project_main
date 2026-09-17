"""Run the unchanged official trainer with device and finite-state observations.

The import hook observes runner construction. It does not change the PPO algorithm.
"""
import json
import builtins
import os
from pathlib import Path
import runpy
import sys

run = Path(os.environ['ASTREX_RUN_DIR'])
lab = Path(os.environ['ASTREX_ISAAC_LAB_ROOT'])
expected = os.environ['ASTREX_RL_DEVICE']
official = lab / 'scripts/reinforcement_learning/rsl_rl/train.py'
stats = {'status': 'FAIL', 'steps': 0, 'finite': True, 'joint_motion': False, 'ros_control': False}
telemetry = (run / 'rl_steps.jsonl').open('w')
runner_seen = False

def observe_runner(runner):
    global runner_seen
    if getattr(runner, '_astrex_observed', False):
        return
    runner._astrex_observed = True
    import torch
    base = runner.env.unwrapped
    stats.update(sim_device=str(base.sim.device), env_device=str(base.device), policy_device=str(runner.device))
    if any(stats[key] != expected for key in ('sim_device', 'env_device', 'policy_device')):
        raise RuntimeError('Actual runtime device mismatch: ' + repr(stats))
    print('ACTUAL_RL_DEVICES', json.dumps(stats), flush=True)
    runner_seen = True
    original = runner.env.step
    previous = None

    def observed_step(actions):
        nonlocal previous
        if not torch.isfinite(actions).all():
            raise RuntimeError('Nonfinite policy action')
        result = original(actions)
        stats['steps'] += 1
        row = {'step': stats['steps'], 'action_min': actions.min().item(), 'action_max': actions.max().item()}
        # Works with other official tasks without requiring a Cartpole attribute.
        assets = base.scene.articulations
        for name, asset in assets.items():
            q, v = asset.data.joint_pos, asset.data.joint_vel
            if not torch.isfinite(q).all() or not torch.isfinite(v).all():
                raise RuntimeError('Nonfinite articulation state: ' + name)
            state = q.detach().cpu()
            if previous is not None and previous.shape == state.shape and not torch.equal(previous, state):
                stats['joint_motion'] = True
            previous = state.clone()
            row[name] = {'position': state.tolist(), 'velocity': v.detach().cpu().tolist()}
        for value in result:
            if isinstance(value, torch.Tensor) and not torch.isfinite(value).all():
                raise RuntimeError('Nonfinite environment output')
        telemetry.write(json.dumps(row, allow_nan=False) + '\n')
        if stats['steps'] % 24 == 0:
            telemetry.flush()
        return result
    runner.env.step = observed_step
    learn = runner.learn

    def observed_learn(*args, **kwargs):
        result = learn(*args, **kwargs)
        if not stats['steps'] or not stats['joint_motion']:
            raise RuntimeError('Trainer did not produce observed GPU motion')
        stats['checkpoints'] = [str(p) for p in run.glob('logs/rsl_rl/**/model_*.pt')]
        if not stats['checkpoints']:
            raise RuntimeError('No checkpoint written')
        stats['status'] = 'PASS'
        telemetry.flush()
        (run / 'rl_result.json').write_text(json.dumps(stats, indent=2))
        print('RL_RESULT', json.dumps(stats), flush=True)
        return result
    runner.learn = observed_learn

original_import = builtins.__import__

def observe_import(name, globals=None, locals=None, fromlist=(), level=0):
    module = original_import(name, globals, locals, fromlist, level)
    if name == 'rsl_rl.runners' and 'OnPolicyRunner' in (fromlist or ()):
        builtins.__import__ = original_import
        cls = module.OnPolicyRunner
        original_init = cls.__init__
        def observed_init(self, *args, **kwargs):
            original_init(self, *args, **kwargs)
            observe_runner(self)
        cls.__init__ = observed_init
    return module

try:
    sys.path.insert(0, str(official.parent))
    sys.argv[0] = str(official)
    builtins.__import__ = observe_import
    runpy.run_path(str(official), run_name='__main__')
    if not runner_seen or not stats['steps'] or not stats['joint_motion']:
        raise RuntimeError('Trainer did not produce observed GPU motion')
    stats['checkpoints'] = [str(p) for p in run.glob('logs/rsl_rl/**/model_*.pt')]
    if not stats['checkpoints']:
        raise RuntimeError('No checkpoint written')
    stats['status'] = 'PASS'
finally:
    builtins.__import__ = original_import
    telemetry.close()
    (run / 'rl_result.json').write_text(json.dumps(stats, indent=2))
    print('RL_RESULT', json.dumps(stats), flush=True)
