"""Highlight green training targets red and tint the rest blue.

Usage:
  python convert_images_modified.py input.jpg -o output.jpg
  python convert_images_modified.py input.jpg -o output.jpg --mode editor

The default mask mode is reliable across changing outdoor illumination.
Editor mode approximates global color sliders; exact slider math is app-specific.
"""
import argparse
from pathlib import Path
import cv2
import numpy as np


def editor_approx(image, hue_degrees=-162, saturation=100, temperature=-100, tint=-100):
    """Approximate editor sliders with white balance followed by HSV edits."""
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    # Temperature cools red and boosts blue; negative tint boosts green.
    temp = temperature / 100.0
    magenta = tint / 100.0
    gains = np.array([1 + 0.30 * temp + 0.12 * magenta,
                      1 - 0.18 * magenta,
                      1 - 0.30 * temp + 0.12 * magenta], dtype=np.float32)
    rgb = np.clip(rgb * gains, 0, 1)
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)  # float hue: 0..360
    hsv[..., 0] = (hsv[..., 0] + hue_degrees) % 360.0
    hsv[..., 1] = np.clip(hsv[..., 1] * (1 + saturation / 100.0), 0, 1)
    return cv2.cvtColor(np.clip(cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB) * 255, 0, 255).astype(np.uint8), cv2.COLOR_RGB2BGR)


def highlight_mines(image, min_hue=77, max_hue=93, min_sat=125, min_val=75,
                    blue_strength=0.90, red_strength=0.98,
                    min_area=20, max_area=2400):
    """Select compact cyan-green mines, not ordinary green grass or turf.

    Calibrated from the uploaded collage: mine HSV hue is about 81-85 on
    OpenCV's 0-179 scale, whereas the green playing field is about 66-70.
    Size filtering rejects isolated cyan noise and broad cyan backgrounds.
    Assumes mines occupy approximately the same pixel scale as the reference.
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower = np.array([min_hue, min_sat, min_val], dtype=np.uint8)
    upper = np.array([max_hue, 255, 255], dtype=np.uint8)
    candidates = cv2.inRange(hsv, lower, upper)
    # Preserve small mine fragments: opening can erase narrow edges.
    # Connected components below remove isolated noise instead.
    candidates = cv2.medianBlur(candidates, 3)
    # Connect nearby mine-colored pixels without recoloring broad backgrounds.
    grouped = cv2.dilate(candidates, np.ones((9, 9), np.uint8))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(grouped, 8)
    mask = np.zeros_like(candidates)
    for idx in range(1, n):
        x, y, w, h, _ = stats[idx]
        region = labels[y:y+h, x:x+w] == idx
        original_region = candidates[y:y+h, x:x+w]
        area = int(np.count_nonzero(original_region[region]))
        if min_area <= area <= max_area:
            roi = mask[y:y+h, x:x+w]
            roi[region & (original_region > 0)] = 255

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY).astype(np.float32)
    blue = np.stack([np.clip(gray * 1.8 + 45, 0, 255),
                     np.clip(gray * 0.30, 0, 255),
                     np.clip(gray * 0.18, 0, 255)], axis=-1)
    red = np.stack([np.clip(gray * 0.12, 0, 255),
                    np.clip(gray * 0.12, 0, 255),
                    np.clip(gray * 1.6 + 85, 0, 255)], axis=-1)
    src = image.astype(np.float32)
    background = src * (1 - blue_strength) + blue * blue_strength
    foreground = src * (1 - red_strength) + red * red_strength
    # Solid red throughout detected mines, including their thin outlines.
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    alpha = (mask.astype(np.float32) / 255)[..., None]
    out = background * (1 - alpha) + foreground * alpha
    return np.clip(out, 0, 255).astype(np.uint8), mask


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('-o', '--output', type=Path, default=Path('output.jpg'))
    parser.add_argument('--mode', choices=['mask', 'editor'], default='mask')
    parser.add_argument('--hue', type=float, default=-162)
    parser.add_argument('--saturation', type=float, default=100)
    parser.add_argument('--temperature', type=float, default=-100)
    parser.add_argument('--tint', type=float, default=-100)
    parser.add_argument('--min-hue', type=int, default=77)
    parser.add_argument('--max-hue', type=int, default=93)
    parser.add_argument('--min-sat', type=int, default=125)
    parser.add_argument('--min-val', type=int, default=77)
    parser.add_argument('--min-area', type=int, default=20)
    parser.add_argument('--max-area', type=int, default=2400)
    parser.add_argument('--save-mask', type=Path)
    args = parser.parse_args()
    image = cv2.imread(str(args.input))
    if image is None:
        parser.error(f'Cannot read image: {args.input}')
    if args.mode == 'editor':
        result = editor_approx(image, args.hue, args.saturation, args.temperature, args.tint)
        mask = None
    else:
        result, mask = highlight_mines(image, args.min_hue, args.max_hue,
                                       args.min_sat, args.min_val, min_area=args.min_area,
                                       max_area=args.max_area)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.output), result):
        parser.error(f'Cannot write: {args.output}')
    if args.save_mask and mask is not None:
        cv2.imwrite(str(args.save_mask), mask)
    print(f'Saved {args.output}')


if __name__ == '__main__':
    main()
