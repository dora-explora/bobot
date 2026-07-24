#!/usr/bin/env python3
"""Small OpenCV box annotator for ball/cone capture sessions."""
import argparse
from pathlib import Path

import cv2

try:
    from tools.annotation_data import (
        Annotation,
        CLASS_NAMES,
        image_paths,
        label_path_for,
        read_annotations,
        write_annotations,
    )
except ModuleNotFoundError:
    from annotation_data import (
        Annotation,
        CLASS_NAMES,
        image_paths,
        label_path_for,
        read_annotations,
        write_annotations,
    )


COLORS = ((64, 230, 64), (0, 150, 255))
WINDOW_NAME = "Bobot Ball/Cone Annotator"


class Annotator:
    def __init__(self, images_root, labels_root, max_width=1400, max_height=850):
        self.images_root = Path(images_root)
        self.labels_root = Path(labels_root)
        self.images = image_paths(self.images_root)
        if not self.images:
            raise ValueError("No JPG, JPEG, or PNG files found under " + str(self.images_root))
        self.max_width = max_width
        self.max_height = max_height
        self.index = self._first_unreviewed()
        self.class_id = 0
        self.image = None
        self.display = None
        self.scale = 1.0
        self.annotations = []
        self.drag_start = None
        self.drag_current = None

    def run(self):
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(WINDOW_NAME, self._mouse)
        self._load()
        while True:
            cv2.imshow(WINDOW_NAME, self._render())
            key = cv2.waitKeyEx(20)
            if key < 0:
                continue
            key &= 0xFF
            if key in (ord("1"), ord("2")):
                self.class_id = key - ord("1")
            elif key in (ord("n"), 10, 13, 32):
                self._move(1)
            elif key == ord("p"):
                self._move(-1)
            elif key in (ord("u"), 8, 127):
                if self.annotations:
                    self.annotations.pop()
            elif key == ord("q"):
                self._save()
                break
        cv2.destroyWindow(WINDOW_NAME)

    def _first_unreviewed(self):
        for index, image_path in enumerate(self.images):
            label_path = label_path_for(
                image_path,
                self.images_root,
                self.labels_root,
            )
            if not label_path.exists():
                return index
        return 0

    def _load(self):
        image_path = self.images[self.index]
        self.image = cv2.imread(str(image_path))
        if self.image is None:
            raise OSError("Could not read " + str(image_path))
        image_height, image_width = self.image.shape[:2]
        self.scale = min(
            1.0,
            self.max_width / float(image_width),
            self.max_height / float(image_height),
        )
        self.annotations = read_annotations(
            label_path_for(image_path, self.images_root, self.labels_root)
        )
        self.drag_start = None
        self.drag_current = None

    def _save(self):
        write_annotations(
            label_path_for(
                self.images[self.index],
                self.images_root,
                self.labels_root,
            ),
            self.annotations,
        )

    def _move(self, direction):
        self._save()
        self.index = max(0, min(len(self.images) - 1, self.index + direction))
        self._load()

    def _mouse(self, event, x, y, _flags, _parameter):
        image_x = int(round(x / self.scale))
        image_y = int(round(y / self.scale))
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drag_start = (image_x, image_y)
            self.drag_current = self.drag_start
        elif event == cv2.EVENT_MOUSEMOVE and self.drag_start is not None:
            self.drag_current = (image_x, image_y)
        elif event == cv2.EVENT_LBUTTONUP and self.drag_start is not None:
            image_height, image_width = self.image.shape[:2]
            annotation = Annotation.from_pixel_box(
                self.class_id,
                self.drag_start[0],
                self.drag_start[1],
                image_x,
                image_y,
                image_width,
                image_height,
            )
            if annotation.width * image_width >= 3 and annotation.height * image_height >= 3:
                self.annotations.append(annotation)
            self.drag_start = None
            self.drag_current = None
        elif event == cv2.EVENT_RBUTTONDOWN:
            self._delete_nearest(image_x, image_y)

    def _delete_nearest(self, x, y):
        if not self.annotations:
            return
        image_height, image_width = self.image.shape[:2]

        def distance(annotation):
            x1, y1, x2, y2 = annotation.pixel_box(image_width, image_height)
            if x1 <= x <= x2 and y1 <= y <= y2:
                return -1.0
            center_x, center_y = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            return (x - center_x) ** 2 + (y - center_y) ** 2

        closest = min(range(len(self.annotations)), key=lambda index: distance(self.annotations[index]))
        self.annotations.pop(closest)

    def _render(self):
        canvas = self.image.copy()
        image_height, image_width = canvas.shape[:2]
        for annotation in self.annotations:
            x1, y1, x2, y2 = annotation.pixel_box(image_width, image_height)
            color = COLORS[annotation.class_id]
            cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                canvas,
                CLASS_NAMES[annotation.class_id],
                (x1, max(18, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA,
            )
        if self.drag_start is not None and self.drag_current is not None:
            cv2.rectangle(
                canvas,
                self.drag_start,
                self.drag_current,
                COLORS[self.class_id],
                1,
            )
        status = (
            str(self.index + 1)
            + "/"
            + str(len(self.images))
            + "  class="
            + CLASS_NAMES[self.class_id]
            + "  1=ball 2=cone  drag=box right-click=delete u=undo n=next p=previous q=save/quit"
        )
        cv2.rectangle(canvas, (0, 0), (image_width, 28), (0, 0, 0), -1)
        cv2.putText(
            canvas,
            status,
            (8, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        if self.scale < 1.0:
            canvas = cv2.resize(
                canvas,
                None,
                fx=self.scale,
                fy=self.scale,
                interpolation=cv2.INTER_AREA,
            )
        return canvas


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", default="datasets/captures")
    parser.add_argument("--labels", default="datasets/labels")
    parser.add_argument("--max-width", type=int, default=1400)
    parser.add_argument("--max-height", type=int, default=850)
    return parser.parse_args()


def main():
    args = parse_args()
    Annotator(
        args.images,
        args.labels,
        args.max_width,
        args.max_height,
    ).run()


if __name__ == "__main__":
    main()
