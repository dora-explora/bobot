from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from tools.annotation_data import (
    Annotation,
    label_path_for,
    read_annotations,
    write_annotations,
)
from tools.prepare_dataset import build_dataset, collect_records, split_records


class AnnotationDataTests(unittest.TestCase):
    def test_pixel_box_round_trip_and_yolo_serialization(self):
        annotation = Annotation.from_pixel_box(1, 10, 20, 50, 60, 100, 100)

        self.assertEqual(annotation.pixel_box(100, 100), (10, 20, 50, 60))
        self.assertEqual(annotation.serialize(), "1 0.300000 0.400000 0.400000 0.400000")

    def test_empty_label_marks_a_reviewed_negative(self):
        with TemporaryDirectory() as temporary_directory:
            label = Path(temporary_directory) / "empty.txt"

            write_annotations(label, [])

            self.assertTrue(label.exists())
            self.assertEqual(read_annotations(label), [])

    def test_invalid_class_is_rejected(self):
        with TemporaryDirectory() as temporary_directory:
            label = Path(temporary_directory) / "bad.txt"
            label.write_text("2 0.5 0.5 0.2 0.2\n", encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "class must be 0 or 1"):
                read_annotations(label)


class DatasetPreparationTests(unittest.TestCase):
    def _make_session(self, images_root, labels_root, name, count=3):
        for index in range(count):
            image = images_root / name / ("frame_" + str(index) + ".jpg")
            image.parent.mkdir(parents=True, exist_ok=True)
            image.write_bytes(b"fake jpeg data")
            label = label_path_for(image, images_root, labels_root)
            write_annotations(label, [Annotation(0, 0.5, 0.5, 0.2, 0.2)])

    def test_three_sessions_do_not_cross_split_boundaries(self):
        with TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            images, labels = root / "captures", root / "labels"
            for session in ("day1", "day2", "day3"):
                self._make_session(images, labels, session)
            records = collect_records(images, labels)

            splits, grouped = split_records(records, seed=10)

            self.assertTrue(grouped)
            session_sets = [
                {record.session for record in splits[split]}
                for split in ("train", "val", "test")
            ]
            self.assertTrue(all(session_sets))
            self.assertFalse(session_sets[0] & session_sets[1])
            self.assertFalse(session_sets[0] & session_sets[2])
            self.assertFalse(session_sets[1] & session_sets[2])

    def test_build_writes_yolo_layout_manifest_and_yaml(self):
        with TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            images, labels = root / "captures", root / "labels"
            for session in ("day1", "day2", "day3"):
                self._make_session(images, labels, session)
            records = collect_records(images, labels)

            splits, _ = build_dataset(records, root / "dataset")

            self.assertEqual(sum(len(items) for items in splits.values()), 9)
            self.assertTrue((root / "dataset" / "data.yaml").exists())
            self.assertTrue((root / "dataset" / "manifest.json").exists())
            self.assertEqual(
                len(list((root / "dataset" / "images").rglob("*.jpg"))),
                9,
            )
            self.assertEqual(
                len(list((root / "dataset" / "labels").rglob("*.txt"))),
                9,
            )

    def test_unreviewed_image_fails_validation(self):
        with TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            image = root / "captures" / "day1" / "frame.jpg"
            image.parent.mkdir(parents=True)
            image.write_bytes(b"fake jpeg data")

            with self.assertRaisesRegex(ValueError, "not reviewed"):
                collect_records(root / "captures", root / "labels")


if __name__ == "__main__":
    unittest.main()
