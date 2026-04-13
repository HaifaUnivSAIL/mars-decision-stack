#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path

import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Render a ROS params file for a fully qualified node name.")
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--node-name", required=True)
    parser.add_argument("--namespace", default="")
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def resolve_node_key(data: dict, node_name: str) -> str:
    for key in data.keys():
        if key == node_name or key == f"/{node_name}" or key.rsplit("/", 1)[-1] == node_name:
            return key
    return node_name


def main() -> int:
    args = parse_args()
    data = yaml.safe_load(args.source.read_text()) or {}
    resolved_key = resolve_node_key(data, args.node_name)
    params = dict(data.get(resolved_key, {}).get("ros__parameters", {}))

    namespace = args.namespace.strip()
    if namespace and not namespace.startswith("/"):
        namespace = f"/{namespace}"
    fq_node_name = f"{namespace}/{args.node_name}" if namespace else f"/{args.node_name}"

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump({fq_node_name: {"ros__parameters": params}}, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
