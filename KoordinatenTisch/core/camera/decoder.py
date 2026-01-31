from __future__ import annotations

import numpy as np


class FrameDecoder:

    @staticmethod
    def decode_mono12packed(u8: np.ndarray, w: int, h: int) -> np.ndarray:
        expected = (w * h * 3) // 2
        triples = u8[:expected].reshape(h, w // 2, 3)
        b0 = triples[:, :, 0].astype(np.uint16)
        b1 = triples[:, :, 1].astype(np.uint16)
        b2 = triples[:, :, 2].astype(np.uint16)
        p0 = (b0 | ((b1 & 0x0F) << 8)) & 0x0FFF
        p1 = ((b1 >> 4) | (b2 << 4)) & 0x0FFF
        out = np.empty((h, w), dtype=np.uint16)
        out[:, 0::2] = p0
        out[:, 1::2] = p1
        return out

    def to_display_u8_copy(self, comp, pf_str: str):
        h, w = comp.height, comp.width
        u8 = comp.data
        total = u8.size

        if w == 0 or h == 0 or total == 0:
            return np.zeros((10, 10), np.uint8), "Empty"

        pf = (pf_str or "").lower()

        if "mono8" in pf:
            return u8.reshape(h, w).copy(), "Mono8 (8b)"

        if "mono12packed" in pf:
            img12 = self.decode_mono12packed(u8, w, h)
            return (img12 >> 4).astype(np.uint8, copy=True), "Mono12Packed (>>4)"

        if "mono12" in pf and "packed" not in pf:
            img16 = np.frombuffer(u8.tobytes(), dtype=np.uint16, count=w * h).reshape(h, w)
            return (img16 >> 4).astype(np.uint8, copy=True), "Mono12 (>>4)"

        if "mono16" in pf:
            img16 = np.frombuffer(u8.tobytes(), dtype=np.uint16, count=w * h).reshape(h, w)
            return (img16 >> 8).astype(np.uint8, copy=True), "Mono16 (>>8)"

        if "mono10" in pf and "packed" not in pf:
            img16 = np.frombuffer(u8.tobytes(), dtype=np.uint16, count=w * h).reshape(h, w)
            return (img16 >> 2).astype(np.uint8, copy=True), "Mono10 (>>2)"

        # Fallback anhand Bytes/Pixel
        bpp = total / (w * h)
        if abs(bpp - 1.0) < 1e-3:
            return u8.reshape(h, w).copy(), "8b(?)"
        if abs(bpp - 2.0) < 1e-3:
            img16 = np.frombuffer(u8.tobytes(), dtype=np.uint16, count=w * h).reshape(h, w)
            return (img16 >> 8).astype(np.uint8, copy=True), "16b(>>8?)"
        if abs(bpp - 1.5) < 1e-2 and (w % 2 == 0):
            img12 = self.decode_mono12packed(u8, w, h)
            return (img12 >> 4).astype(np.uint8, copy=True), "12p(>>4?)"

        return u8.reshape(h, -1)[:, :w].copy(), f"Unknown bpp={bpp:.2f}"
