#!/usr/bin/env python3

"""Generate dimensionally accurate A4 checkerboard and ArUco print files."""

from pathlib import Path

from PIL import Image, ImageDraw, ImageFont
from reportlab.lib.pagesizes import A4, landscape
from reportlab.lib.units import mm
from reportlab.lib.utils import ImageReader
from reportlab.pdfgen.canvas import Canvas


OUTPUT_DIR = Path(__file__).resolve().parent
DPI = 300
MM_PER_INCH = 25.4


def mm_to_px(value_mm: float) -> int:
    return round(value_mm / MM_PER_INCH * DPI)


def generate_checkerboard() -> None:
    columns = 10
    rows = 7
    square_mm = 25.0
    board_width_mm = columns * square_mm
    board_height_mm = rows * square_mm

    pdf_path = OUTPUT_DIR / 'checkerboard_9x6_25mm_a4.pdf'
    page_width, page_height = landscape(A4)
    origin_x = (page_width - board_width_mm * mm) / 2.0
    origin_y = (page_height - board_height_mm * mm) / 2.0
    pdf = Canvas(str(pdf_path), pagesize=(page_width, page_height))
    pdf.setFillColorRGB(0.0, 0.0, 0.0)
    for row in range(rows):
        for column in range(columns):
            if (row + column) % 2 == 0:
                pdf.rect(
                    origin_x + column * square_mm * mm,
                    origin_y + row * square_mm * mm,
                    square_mm * mm,
                    square_mm * mm,
                    stroke=0,
                    fill=1,
                )
    pdf.showPage()
    pdf.save()

    page_width_px = mm_to_px(297.0)
    page_height_px = mm_to_px(210.0)
    origin_x_px = (page_width_px - mm_to_px(board_width_mm)) // 2
    origin_y_px = (page_height_px - mm_to_px(board_height_mm)) // 2
    page = Image.new('L', (page_width_px, page_height_px), 255)
    draw = ImageDraw.Draw(page)
    for row in range(rows):
        for column in range(columns):
            if (row + column) % 2 == 0:
                left = origin_x_px + mm_to_px(column * square_mm)
                top = origin_y_px + mm_to_px(row * square_mm)
                right = origin_x_px + mm_to_px((column + 1) * square_mm)
                bottom = origin_y_px + mm_to_px((row + 1) * square_mm)
                draw.rectangle((left, top, right - 1, bottom - 1), fill=0)
    page.save(
        OUTPUT_DIR / 'checkerboard_9x6_25mm_a4_300dpi.png',
        dpi=(DPI, DPI),
    )


def get_font(size: int):
    font_path = Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
    if font_path.exists():
        return ImageFont.truetype(str(font_path), size=size)
    return ImageFont.load_default()


def draw_aruco_marker(marker_id: int, marker_source_px: int) -> Image.Image:
    import cv2

    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
    marker = cv2.aruco.drawMarker(
        dictionary, marker_id, marker_source_px, borderBits=1)
    return Image.fromarray(marker, mode='L')


def generate_aruco_page(
        marker_id: int,
        marker_size_mm: float,
        marker_image: Image.Image | None = None) -> None:
    output_dir = OUTPUT_DIR / 'aruco_5x5_50'
    output_dir.mkdir(parents=True, exist_ok=True)
    marker_source_px = 1600
    size_label = f'{marker_size_mm:g}'
    if marker_image is None:
        marker_image = draw_aruco_marker(marker_id, marker_source_px)
    marker_image = marker_image.convert('L')
    stem = f'aruco_5x5_50_id_{marker_id}_{size_label}mm_a4'

    page_width, page_height = A4
    marker_points = marker_size_mm * mm
    origin_x = (page_width - marker_points) / 2.0
    origin_y = (page_height - marker_points) / 2.0
    pdf = Canvas(
        str(output_dir / f'{stem}.pdf'),
        pagesize=A4,
    )
    pdf.drawImage(
        ImageReader(marker_image),
        origin_x,
        origin_y,
        width=marker_points,
        height=marker_points,
        preserveAspectRatio=True,
        mask='auto',
    )
    pdf.setFont('Helvetica', 11)
    label = (
        f'DICT_5X5_50  ID {marker_id}  |  '
        f'outer size {size_label} mm')
    pdf.drawCentredString(
        page_width / 2.0,
        origin_y - 12.0 * mm,
        label,
    )
    pdf.showPage()
    pdf.save()

    page_width_px = mm_to_px(210.0)
    page_height_px = mm_to_px(297.0)
    marker_px = mm_to_px(marker_size_mm)
    page = Image.new('L', (page_width_px, page_height_px), 255)
    marker_on_page = marker_image.resize(
        (marker_px, marker_px), Image.NEAREST)
    origin_x_px = (page_width_px - marker_px) // 2
    origin_y_px = (page_height_px - marker_px) // 2
    page.paste(marker_on_page, (origin_x_px, origin_y_px))
    draw = ImageDraw.Draw(page)
    font = get_font(34)
    label_box = draw.textbbox((0, 0), label, font=font)
    label_width = label_box[2] - label_box[0]
    draw.text(
        ((page_width_px - label_width) // 2,
         origin_y_px + marker_px + mm_to_px(10.0)),
        label,
        fill=0,
        font=font,
    )
    page.save(output_dir / f'{stem}_300dpi.png', dpi=(DPI, DPI))


def generate_aruco_pages() -> None:
    for marker_id in range(5):
        generate_aruco_page(marker_id, 160.0)
    generate_aruco_page(0, 80.0)


def main() -> None:
    generate_checkerboard()
    generate_aruco_pages()


if __name__ == '__main__':
    main()
