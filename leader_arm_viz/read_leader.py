import time
import numpy as np

import lerobot_telleoperator_violin as violin_mod
from lerobot.teleoperators.config import TeleoperatorConfig

cfg_cls = None
for obj in vars(violin_mod).values():
    if isinstance(obj, type) and issubclass(obj, TeleoperatorConfig) and obj is not TeleoperatorConfig:
        cfg_cls = obj
        break
assert cfg_cls is not None, "Could not find a TeleoperatorConfig in lerobot_teleoperator_violin"

cfg = cfg_cls(port="/dev/ttyUSB0", id="violin_leader_arm")
teleop = cfg_cls.__name__.removesuffix("Config")
teleop_cls = getattr(violin_mod, teleop)
dev = teleop_cls(cfg)
dev.connect()

try:
    while True:
        a = dev.get_action()
        # print a compact view
        keys = sorted(a.keys())
        vals = []
        for k in keys:
            v = a[k]
            if hasattr(v, "detach"): v = v.detach().cpu().numpy()
            v = np.array(v).reshape(-1)[0]
            vals.append(float(v))
        print(list(zip(keys, vals)))
        time.sleep(0.02) # 50 Hz
finally:
    dev.disconnect()