"""KNN ball colour: the model only finds balls, the point cloud names their colour.

Active when LARC_COLOR=knn (dispatcher.py VISION_MODE = "knn").
Edit the constants below; nothing else needs to change.
"""
from __future__ import annotations
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# box = sample a circle inside each bounding box; seg = average the whole mask.
SOURCE = 'box'
WEIGHTS = {'box': 'models/single_class_box_V1.pt',
           'seg': 'models/single_class_seg_V1.pt'}
CLOUD = 'models/curated_cloud.json'
K = 15
# Box mode: sampled circle radius as a fraction of half the box's short side.
# 1.0 = touches the box edges; smaller keeps out the background and the rim shadow.
CIRCLE_FRACTION = 0.6

def bgr_to_lab(pixels):
    """uint8 BGR (N,3) -> CIELAB (N,3), the same scale the cloud was saved in."""
    import cv2
    import numpy as np
    px = np.asarray(pixels, dtype=np.float32).reshape(-1, 1, 3) / 255
    return cv2.cvtColor(px, cv2.COLOR_BGR2Lab).reshape(-1, 3)

class KnnColor:
    def __init__(self, path=None, k=K):
        import numpy as np
        data = json.loads(Path(path or ROOT / CLOUD).read_text())
        if data.get('space') != 'CIELAB':
            raise ValueError(f"{path or CLOUD}: expected space CIELAB, got {data.get('space')}")
        removed = set(data.get('removed_cloud_indices', []))
        points = [p for p in data['points'] if p['cloud_index'] not in removed]
        if len(points) < k:
            raise ValueError(f'Point cloud has {len(points)} points, fewer than K={k}')
        self.lab = np.array([p['lab'] for p in points], dtype=np.float32)
        self.ids = np.array([p['class_id'] for p in points])
        self.names = {int(i): n for i, n in data['names'].items()}
        self.k = k

    def classify(self, lab):
        """Majority vote of the K nearest cloud points to one Lab colour."""
        import numpy as np
        d = ((self.lab - np.asarray(lab, dtype=np.float32)) ** 2).sum(axis=1)
        votes = np.bincount(self.ids[np.argpartition(d, self.k)[:self.k]])
        return self.names[int(votes.argmax())]

    def name(self, frame, box, polygon=None):
        """Colour name for one detection: mask average (seg) or circle average (box)."""
        import cv2
        import numpy as np
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        if polygon is not None and len(polygon) >= 3:
            cv2.fillPoly(mask, [np.asarray(polygon, dtype=np.int32)], 255)
        else:
            x1, y1, x2, y2 = box
            r = max(1, int(min(x2 - x1, y2 - y1) / 2 * CIRCLE_FRACTION))
            cv2.circle(mask, (int((x1 + x2) / 2), int((y1 + y2) / 2)), r, 255, -1)
        pixels = frame[mask > 0]
        if not len(pixels):
            return None
        return self.classify(bgr_to_lab(pixels).mean(axis=0))
