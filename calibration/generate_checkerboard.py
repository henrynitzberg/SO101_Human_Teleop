import argparse
import os

from PIL import Image, ImageDraw, ImageFont

LETTER_WIDTH_IN = 8.5
LETTER_HEIGHT_IN = 11.0
LABEL_HEIGHT_IN = 0.9  # reserved below the board for the printed label


def choose_layout(squares_cols, squares_rows, margin_in):
    """Pick whichever of portrait/landscape yields the larger square size."""
    orientations = {
        "portrait": (LETTER_WIDTH_IN, LETTER_HEIGHT_IN),
        "landscape": (LETTER_HEIGHT_IN, LETTER_WIDTH_IN),
    }
    best = None
    for name, (page_w, page_h) in orientations.items():
        usable_w = page_w - 2 * margin_in
        usable_h = page_h - 2 * margin_in - LABEL_HEIGHT_IN
        if usable_w <= 0 or usable_h <= 0:
            continue
        square_size_in = min(usable_w / squares_cols, usable_h / squares_rows)
        if best is None or square_size_in > best[1]:
            best = (name, square_size_in, page_w, page_h)
    if best is None:
        raise ValueError("Board doesn't fit on Letter paper with the given margin.")
    return best


def render_checkerboard(
    squares_cols, squares_rows, square_size_in, page_w_in, page_h_in, margin_in, dpi, label_lines
):
    page_w_px = round(page_w_in * dpi)
    page_h_px = round(page_h_in * dpi)
    margin_px = round(margin_in * dpi)
    square_px = square_size_in * dpi

    image = Image.new("RGB", (page_w_px, page_h_px), "white")
    draw = ImageDraw.Draw(image)

    board_w_px = squares_cols * square_px
    offset_x = (page_w_px - board_w_px) / 2
    offset_y = margin_px

    xs = [round(offset_x + c * square_px) for c in range(squares_cols + 1)]
    ys = [round(offset_y + r * square_px) for r in range(squares_rows + 1)]

    for r in range(squares_rows):
        for c in range(squares_cols):
            if (r + c) % 2 == 0:
                draw.rectangle([xs[c], ys[r], xs[c + 1], ys[r + 1]], fill="black")

    font = ImageFont.load_default(size=max(14, dpi // 15))
    text_y = ys[-1] + int(0.2 * dpi)
    line_gap = int(0.28 * dpi)
    for i, line in enumerate(label_lines):
        bbox = draw.textbbox((0, 0), line, font=font)
        text_w = bbox[2] - bbox[0]
        draw.text(((page_w_px - text_w) / 2, text_y + i * line_gap), line, fill="black", font=font)

    return image


def main():
    parser = argparse.ArgumentParser(
        description="Generate a print-ready checkerboard for stereo camera calibration."
    )
    parser.add_argument("--board-cols", type=int, default=9, help="Inner corner columns.")
    parser.add_argument("--board-rows", type=int, default=6, help="Inner corner rows.")
    parser.add_argument("--margin-in", type=float, default=0.5)
    parser.add_argument("--dpi", type=int, default=300)
    parser.add_argument("--out", type=str, default=None)
    args = parser.parse_args()

    squares_cols = args.board_cols + 1
    squares_rows = args.board_rows + 1

    orientation, square_size_in, page_w_in, page_h_in = choose_layout(
        squares_cols, squares_rows, args.margin_in
    )
    square_size_mm = square_size_in * 25.4
    square_size_m = square_size_in * 0.0254

    out_path = args.out
    if out_path is None:
        out_path = f"calibration/data/checkerboard_{args.board_cols}x{args.board_rows}_{square_size_mm:.1f}mm.png"
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)

    label_lines = [
        f"{args.board_cols}x{args.board_rows} inner corners, {square_size_mm:.2f}mm squares ({orientation})",
        "Print at Actual Size / 100% -- do NOT use Fit to Page",
        f"calibrate_stereo.py --square-size {square_size_m:.5f}",
    ]

    image = render_checkerboard(
        squares_cols, squares_rows, square_size_in, page_w_in, page_h_in, args.margin_in, args.dpi, label_lines
    )
    image.save(out_path, dpi=(args.dpi, args.dpi))

    print(f"Wrote {out_path}")
    print(f"Orientation: {orientation}")
    print(f"Square size: {square_size_mm:.2f}mm ({square_size_m:.5f}m)")
    print(f"Pass --square-size {square_size_m:.5f} to calibrate_stereo.py if it differs from its default.")


if __name__ == "__main__":
    main()
