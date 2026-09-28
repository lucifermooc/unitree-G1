#!/usr/bin/env python3
"""生成离线回放配置：读 <src.yaml>，关闭 UI，按 段.键=值 覆盖。用法: _cfg.py <src.yaml> <out.yaml> 段.键=值 ..."""
import sys
import yaml

src, out, *ovr = sys.argv[1:]
c = yaml.safe_load(open(src))
c['system'].update(with_ui=False, with_2dui=False)
for o in ovr:
    k, v = o.split('=', 1)
    s, key = k.split('.', 1)
    c.setdefault(s, {})[key] = yaml.safe_load(v)
yaml.safe_dump(c, open(out, 'w'))
