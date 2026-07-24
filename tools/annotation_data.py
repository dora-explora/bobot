"""Shared YOLO annotation parsing and validation."""
from dataclasses import dataclass
from pathlib import Path


CLASS_NAMES = ("ball", "cone")
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png"}


@dataclass
class Annotation:
    class_id: int
    x_center: float
    y_center: float
    width: float
    height: float

    @classmethod
    def from_pixel_box(cls, class_id, x1, y1, x2, y2, image_width, image_height):
        first_x = min(image_width, max(0, x1))
        second_x = min(image_width, max(0, x2))
        first_y = min(image_height, max(0, y1))
        second_y = min(image_height, max(0, y2))
        left, right = sorted((first_x, second_x))
        top, bottom = sorted((first_y, second_y))
        return cls(
            class_id=class_id,
            x_center=(left + right) / 2.0 / image_width,
            y_center=(top + bottom) / 2.0 / image_height,
            width=(right - left) / image_width,
            height=(bottom - top) / image_height,
        )

    def pixel_box(self, image_width, image_height):
        half_width = self.width * image_width / 2.0
        half_height = self.height * image_height / 2.0
        center_x = self.x_center * image_width
        center_y = self.y_center * image_height
        return (
            int(round(center_x - half_width)),
            int(round(center_y - half_height)),
            int(round(center_x + half_width)),
            int(round(center_y + half_height)),
        )

    def serialize(self):
        return "{} {:.6f} {:.6f} {:.6f} {:.6f}".format(
            self.class_id,
            self.x_center,
            self.y_center,
            self.width,
            self.height,
        )


def image_paths(root):
    root = Path(root)
    return sorted(
        path
        for path in root.rglob("*")
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
    )


def label_path_for(image_path, image_root, label_root):
    relative = Path(image_path).relative_to(Path(image_root))
    return Path(label_root) / relative.with_suffix(".txt")


def read_annotations(path):
    path = Path(path)
    if not path.exists():
        return []
    annotations = []
    for line_number, raw_line in enumerate(
        path.read_text(encoding="utf-8").splitlines(),
        start=1,
    ):
        line = raw_line.strip()
        if not line:
            continue
        fields = line.split()
        if len(fields) != 5:
            raise ValueError(str(path) + ":" + str(line_number) + " must have 5 fields")
        try:
            class_id = int(fields[0])
            values = [float(value) for value in fields[1:]]
        except ValueError as error:
            raise ValueError(
                str(path) + ":" + str(line_number) + " contains a non-numeric field"
            ) from error
        annotation = Annotation(class_id, *values)
        validate_annotation(annotation, path, line_number)
        annotations.append(annotation)
    return annotations


def write_annotations(path, annotations):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    text = "\n".join(annotation.serialize() for annotation in annotations)
    path.write_text(text + ("\n" if text else ""), encoding="utf-8")


def validate_annotation(annotation, path="<annotation>", line_number=1):
    if not 0 <= annotation.class_id < len(CLASS_NAMES):
        raise ValueError(
            str(path) + ":" + str(line_number) + " class must be 0 or 1"
        )
    coordinates = (
        annotation.x_center,
        annotation.y_center,
        annotation.width,
        annotation.height,
    )
    if any(value < 0.0 or value > 1.0 for value in coordinates):
        raise ValueError(
            str(path) + ":" + str(line_number) + " coordinates must be normalized"
        )
    if annotation.width <= 0.0 or annotation.height <= 0.0:
        raise ValueError(
            str(path) + ":" + str(line_number) + " box size must be positive"
        )
