#!/usr/bin/env python3
"""Validate annotations and build session-aware YOLO train/val/test splits."""
import argparse
from collections import defaultdict
from dataclasses import dataclass
import json
from pathlib import Path
import random
import shutil

try:
    from tools.annotation_data import (
        CLASS_NAMES,
        image_paths,
        label_path_for,
        read_annotations,
    )
except ModuleNotFoundError:
    from annotation_data import (
        CLASS_NAMES,
        image_paths,
        label_path_for,
        read_annotations,
    )


SPLITS = ("train", "val", "test")


@dataclass(frozen=True)
class Record:
    image: Path
    label: Path
    session: str


def collect_records(images_root, labels_root):
    images_root = Path(images_root)
    labels_root = Path(labels_root)
    records = []
    missing = []
    for image in image_paths(images_root):
        label = label_path_for(image, images_root, labels_root)
        if not label.exists():
            missing.append(label)
            continue
        read_annotations(label)
        relative = image.relative_to(images_root)
        session = relative.parts[0] if len(relative.parts) > 1 else "default"
        records.append(Record(image=image, label=label, session=session))
    if missing:
        examples = ", ".join(str(path) for path in missing[:5])
        raise ValueError(
            str(len(missing))
            + " images are not reviewed; missing label files include: "
            + examples
        )
    if not records:
        raise ValueError("No reviewed images found under " + str(images_root))
    return records


def split_records(records, seed=42):
    by_session = defaultdict(list)
    for record in records:
        by_session[record.session].append(record)
    for session_records in by_session.values():
        session_records.sort(key=lambda item: item.image.name)

    sessions = sorted(by_session)
    if len(sessions) >= 3:
        random.Random(seed).shuffle(sessions)
        desired = {
            "train": len(records) * 0.70,
            "val": len(records) * 0.20,
            "test": len(records) * 0.10,
        }
        result = {split: [] for split in SPLITS}
        for split, session in zip(SPLITS, sessions[:3]):
            result[split].extend(by_session[session])
        for session in sessions[3:]:
            split = max(
                SPLITS,
                key=lambda name: desired[name] - len(result[name]),
            )
            result[split].extend(by_session[session])
        return result, True

    result = {split: [] for split in SPLITS}
    for session in sessions:
        session_records = by_session[session]
        count = len(session_records)
        train_end = max(1, int(round(count * 0.70)))
        val_end = max(train_end + 1, int(round(count * 0.90)))
        train_end = min(train_end, count)
        val_end = min(val_end, count)
        result["train"].extend(session_records[:train_end])
        result["val"].extend(session_records[train_end:val_end])
        result["test"].extend(session_records[val_end:])
    if not result["val"] or not result["test"]:
        raise ValueError(
            "At least 3 reviewed images are needed; 3 separate capture sessions are strongly recommended"
        )
    return result, False


def build_dataset(records, output, seed=42, replace=False):
    output = Path(output)
    if output.exists():
        if not replace:
            raise FileExistsError(
                str(output) + " exists; pass --replace to rebuild generated splits"
            )
        shutil.rmtree(output)

    splits, grouped_by_session = split_records(records, seed)
    manifest = {
        "classes": list(CLASS_NAMES),
        "seed": seed,
        "grouped_by_session": grouped_by_session,
        "splits": {},
    }
    for split, split_records_list in splits.items():
        image_directory = output / "images" / split
        label_directory = output / "labels" / split
        image_directory.mkdir(parents=True, exist_ok=True)
        label_directory.mkdir(parents=True, exist_ok=True)
        manifest["splits"][split] = []
        for index, record in enumerate(split_records_list):
            stem = _safe_name(record.session) + "_" + str(index).zfill(5) + "_" + record.image.stem
            image_destination = image_directory / (stem + record.image.suffix.lower())
            label_destination = label_directory / (stem + ".txt")
            shutil.copy2(record.image, image_destination)
            shutil.copy2(record.label, label_destination)
            manifest["splits"][split].append(
                {
                    "image": str(image_destination.relative_to(output)),
                    "source": str(record.image),
                    "session": record.session,
                }
            )

    data_yaml = (
        "path: " + str(output.resolve()) + "\n"
        + "train: images/train\n"
        + "val: images/val\n"
        + "test: images/test\n"
        + "names:\n"
        + "  0: ball\n"
        + "  1: cone\n"
    )
    (output / "data.yaml").write_text(data_yaml, encoding="utf-8")
    (output / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return splits, grouped_by_session


def _safe_name(value):
    safe = "".join(character if character.isalnum() else "_" for character in value)
    return safe.strip("_") or "session"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", default="datasets/captures")
    parser.add_argument("--labels", default="datasets/labels")
    parser.add_argument("--output", default="datasets/ball_cone")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--replace", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    records = collect_records(args.images, args.labels)
    splits, grouped = build_dataset(
        records,
        args.output,
        seed=args.seed,
        replace=args.replace,
    )
    print(
        "Prepared "
        + str(sum(len(items) for items in splits.values()))
        + " images: "
        + ", ".join(name + "=" + str(len(splits[name])) for name in SPLITS)
    )
    if not grouped:
        print(
            "WARNING: fewer than 3 sessions; used contiguous within-session splits. "
            "Capture more sessions before final training."
        )


if __name__ == "__main__":
    main()
