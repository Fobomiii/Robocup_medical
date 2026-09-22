#!/usr/bin/env python3
"""
生成医疗机器人赛事用的二维码和条形码打印PDF。

布局：每页A4纸放一个码，居中，下方标注名称。
尺寸（按规则要求）：
  - 二维码：10cm x 10cm
  - 条形码：长 20cm（CODE128）

要点：
  * 条码不打印人眼可读数字（规则：场地内条码和二维码均不显示数字和文字），
    同时避免数字与条条重叠。
  * 页面标注用 PyMuPDF 内置 CJK 字体，避免嵌入整套字体文件。
  * 保存时启用 deflate+garbage，否则图像以未压缩方式存储，文件会膨胀到上百MB。
"""
import io
import os
import fitz                       # PyMuPDF
import qrcode
from barcode import Code128
from barcode.writer import ImageWriter
from PIL import Image

CM_TO_PT = 72 / 2.54
A4_W, A4_H = 595.276, 841.89     # A4 in points: 210 x 297 mm

QR_SIZE_CM = 10.0
BARCODE_W_CM = 20.0
BARCODE_H_CM = 5.0
RENDER_DPI = 600
LABEL_PT = 12

QR_CODES = ['11', '13', '31', '33']
BARCODES = ['6946522463487', '6921361255288', '6911345321863',
            '6944060407291', '6906841121017', '6938237700261']

# 条码渲染标定：module_width=1.455mm 时条码宽恰好 20.0cm @ 600DPI
BARCODE_CALIB_MW = 1.455
BARCODE_CALIB_W_CM = 20.0


def make_qr(data, size_cm, dpi=RENDER_DPI):
    """按模块数反推 box_size，使二维码落在精确的物理尺寸上，不做重采样。"""
    qr = qrcode.QRCode(error_correction=qrcode.constants.ERROR_CORRECT_M, border=4)
    qr.add_data(data)
    qr.make(fit=True)
    modules = qr.modules_count + 2 * qr.border
    target_px = int(round(size_cm / 2.54 * dpi))
    qr.box_size = max(1, target_px // modules)
    return qr.make_image(fill_color='black', back_color='white').convert('RGB')


def make_barcode(data, width_cm, height_cm, dpi=RENDER_DPI):
    """渲染 CODE128。不含人眼可读数字，因此图像自上而下都是条码本体，
    不会再出现数字与条条重叠。"""
    module_width_mm = BARCODE_CALIB_MW * (width_cm / BARCODE_CALIB_W_CM)

    writer = ImageWriter()
    writer.dpi = dpi
    opts = {
        'module_width': module_width_mm,
        'module_height': height_cm * 10.0,   # mm
        'quiet_zone': 10.0,                  # mm
        'write_text': False,                 # 规则要求不显示数字和文字
        'dpi': dpi,
    }
    buf = io.BytesIO()
    Code128(data, writer=writer).write(buf, options=opts)
    buf.seek(0)
    return Image.open(buf).convert('RGB')


def place(page, pil_img, label):
    """按图像自身在 RENDER_DPI 下的物理尺寸插入，水平居中，标注置于下方。"""
    buf = io.BytesIO()
    pil_img.save(buf, format='PNG')
    stream = buf.getvalue()

    w_px, h_px = pil_img.size
    w_pt = w_px * 72.0 / RENDER_DPI
    h_pt = h_px * 72.0 / RENDER_DPI

    x = (A4_W - w_pt) / 2
    y = (A4_H - h_pt) / 2 - 20
    page.insert_image(fitz.Rect(x, y, x + w_pt, y + h_pt), stream=stream)

    # 用 insert_htmlbox 做真正的居中排版，避免手算字宽造成字距怪异
    box = fitz.Rect(0, y + h_pt + 10, A4_W, y + h_pt + 40)
    page.insert_htmlbox(
        box,
        f'<div style="text-align:center; font-size:{LABEL_PT}pt; '
        f'font-family:sans-serif;">{label}</div>',
    )


def main():
    pdf = fitz.open()
    entries = []

    for v in QR_CODES:
        pg = pdf.new_page(width=A4_W, height=A4_H)
        place(pg, make_qr(v, QR_SIZE_CM), f'药箱二维码 {v}')
        entries.append(('QR', v))

    for v in BARCODES:
        pg = pdf.new_page(width=A4_W, height=A4_H)
        place(pg, make_barcode(v, BARCODE_W_CM, BARCODE_H_CM), f'药品条形码 {v}')
        entries.append(('CODE128', v))

    out = '医疗机器人赛事_二维码条形码打印版_v2.pdf'
    # deflate 压缩图像流，garbage 清除无用对象；否则文件可达上百MB
    pdf.save(out, garbage=4, deflate=True, clean=True)
    pdf.close()

    size_mb = os.path.getsize(out) / 1e6
    print(f'generated {out}: {len(entries)} pages, {size_mb:.2f} MB')
    return out, entries


if __name__ == '__main__':
    main()
