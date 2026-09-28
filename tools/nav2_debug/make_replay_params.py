#!/usr/bin/env python3
"""把 nav2_params.yaml 的 local_costmap 段转成"独立 costmap 节点"能用的参数文件。

    python3 make_replay_params.py <nav2_params.yaml> <out.yaml> [--set a.b=v ...]

用途：离线回放时跑 `ros2 run nav2_costmap_2d nav2_costmap_2d`（节点名固定是 /costmap），
它的参数根键必须是 costmap:，而不是 local_costmap: local_costmap:。
顺手做三件事：
  * 去掉 keepout_layer（依赖 costmap_filter_info，离线没有，也和本次噪声无关）
  * use_sim_time: true（跟 ros2 bag play --clock 对齐）
  * --set 覆盖任意参数，用来做 A/B：--set stvl_voxel_layer.voxel_decay=1.0
"""
import sys, yaml, argparse


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('src'); ap.add_argument('out')
    ap.add_argument('--set', action='append', default=[], metavar='KEY=VAL')
    ap.add_argument('--keep-keepout', action='store_true')
    ap.add_argument('--section', default='local_costmap', choices=['local_costmap', 'global_costmap'],
                    help='回放哪张代价地图（global 需要回放 /map 给 static_layer）')
    args = ap.parse_args()

    root = yaml.safe_load(open(args.src))
    p = root[args.section][args.section]['ros__parameters']
    p['use_sim_time'] = True
    if not args.keep_keepout:
        p['plugins'] = [x for x in p['plugins'] if x != 'keepout_layer']
        p.pop('keepout_layer', None)

    def cast(v):
        for f in (int, float):
            try:
                return f(v)
            except ValueError:
                pass
        if v.lower() in ('true', 'false'):
            return v.lower() == 'true'
        if v.startswith('[') and v.endswith(']'):
            return [cast(x.strip()) for x in v[1:-1].split(',') if x.strip()]
        return v

    for s in args.set:
        k, v = s.split('=', 1)
        d = p
        parts = k.split('.')
        for part in parts[:-1]:
            d = d.setdefault(part, {})
        d[parts[-1]] = cast(v)
        print(f'  覆盖 {k} = {d[parts[-1]]!r}')

    yaml.safe_dump({'costmap': {'ros__parameters': p}}, open(args.out, 'w'),
                   default_flow_style=False, allow_unicode=True, sort_keys=False)
    print(f'写出 {args.out}  plugins={p["plugins"]}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
