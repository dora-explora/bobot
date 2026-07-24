#!/usr/bin/env python3
"""Fine-tune YOLO26n and export an end-to-end ONNX ball/cone detector."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil


def automatic_device():
    import torch

    return "0" if torch.cuda.is_available() else "cpu"


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as model_file:
        for chunk in iter(lambda: model_file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def train(args):
    from ultralytics import YOLO

    device = automatic_device() if args.device == "auto" else args.device
    model = YOLO(args.resume or args.model)
    if args.resume:
        model.train(resume=True, device=device)
    else:
        model.train(
            data=str(Path(args.data).resolve()),
            epochs=args.epochs,
            imgsz=args.imgsz,
            batch=args.batch,
            workers=args.workers,
            device=device,
            project=args.project,
            name=args.name,
            seed=args.seed,
            patience=args.patience,
            cache=args.cache,
            hsv_h=0.03,
            hsv_s=0.50,
            hsv_v=0.50,
            degrees=5.0,
            translate=0.10,
            scale=0.40,
            fliplr=0.50,
            mosaic=0.50,
            close_mosaic=min(10, max(0, args.epochs // 5)),
        )

    best_path = Path(model.trainer.best)
    if not best_path.exists():
        raise FileNotFoundError("Training did not produce " + str(best_path))
    best_model = YOLO(str(best_path))
    best_model.val(
        data=str(Path(args.data).resolve()),
        split="test",
        imgsz=args.imgsz,
        device=device,
    )
    exported_path = Path(
        best_model.export(
            format="onnx",
            imgsz=args.imgsz,
            batch=1,
            dynamic=False,
            simplify=True,
            max_det=args.max_det,
        )
    )
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(exported_path, output)
    manifest_path = output.with_suffix(".json")
    manifest = {
        "format": "yolo26-e2e-onnx",
        "input_width": args.imgsz,
        "input_height": args.imgsz,
        "classes": {"0": "ball", "1": "cone"},
        "max_detections": args.max_det,
        "sha256": sha256(output),
        "trained_at": datetime.now(timezone.utc).isoformat(),
        "training_model": args.model,
        "training_data": str(Path(args.data).resolve()),
        "best_checkpoint": str(best_path.resolve()),
    }
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print("Pi model: " + str(output))
    print("Manifest: " + str(manifest_path))
    print("Device: " + device)
    return output, manifest_path


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", default="datasets/ball_cone/data.yaml")
    parser.add_argument("--model", default="yolo26n.pt")
    parser.add_argument("--resume", default="")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=512)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--project", default="runs/ball_cone")
    parser.add_argument("--name", default="train")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--patience", type=int, default=30)
    parser.add_argument("--cache", default=False)
    parser.add_argument("--max-det", type=int, default=100)
    parser.add_argument("--output", default="models/ball_cone.onnx")
    return parser.parse_args()


def main():
    train(parse_args())


if __name__ == "__main__":
    main()

