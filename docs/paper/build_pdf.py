#!/usr/bin/env python3
"""Build A_FRANKENSTEIN_EXPERIENCE.pdf from the Markdown, in the Skipper Flipper look
(aged paper, film-strip borders on every page, Chewy headings, typewriter captions).

Needs: pandoc (sudo apt install pandoc) and Playwright's Chromium
(pip install playwright && python3 -m playwright install chromium).
Run from anywhere: python3 docs/paper/build_pdf.py
Fonts (Apache-2.0) are in fonts/, images in images/.
"""
import os, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
MD = os.path.join(HERE, "A_FRANKENSTEIN_EXPERIENCE.md")
PDF = os.path.join(HERE, "A_FRANKENSTEIN_EXPERIENCE.pdf")

CSS = r"""
@font-face{font-family:Chewy;src:url(fonts/chewy.woff2)}
@font-face{font-family:Typewriter;src:url(fonts/special-elite.woff2)}
@page{size:A4;margin:0}
html,body{margin:0;padding:0;background:transparent;-webkit-print-color-adjust:exact;print-color-adjust:exact}
/* the paper and the film strip, repeated on every printed page */
.bg{position:fixed;left:0;top:0;width:210mm;height:297mm;background:#e9e6dc;z-index:-2}
.strip{position:fixed;left:0;width:210mm;height:7mm;z-index:-1;background-color:#141414;
  background-image:url("data:image/svg+xml;utf8,<svg xmlns='http://www.w3.org/2000/svg' width='50' height='70'><rect x='14' y='22' width='22' height='26' rx='5' fill='%23e9e6dc'/></svg>");
  background-size:5mm 7mm;background-repeat:repeat-x}
.strip.t{top:0}.strip.b{bottom:0}
.edge{position:fixed;top:0;height:297mm;width:2.2mm;background:#141414;z-index:-1}.edge.l{left:7mm}.edge.r{right:7mm}
body{font:10.6pt/1.5 "DejaVu Serif",Georgia,serif;color:#141414}
h1{font:400 40pt/1 Chewy,"DejaVu Sans",sans-serif;margin:4mm 0 1mm;letter-spacing:.5px}
h2{font:400 19pt/1.15 Chewy,"DejaVu Sans",sans-serif;margin:7mm 0 2mm;padding:1mm 3mm;background:#141414;color:#e9e6dc;
  break-after:avoid;page-break-after:avoid}
h1+h2{background:none;color:#141414;padding:0;font-size:21pt;margin-top:0}
h3{font:400 14pt Chewy,"DejaVu Sans",sans-serif;margin:5mm 0 1mm;border-bottom:2.5px solid #141414;break-after:avoid}
p,li{orphans:3;widows:3}
a{color:#14417a;text-decoration:none;border-bottom:1px dotted #14417a}
code{font:8.8pt "DejaVu Sans Mono",monospace;background:#f7f5ee;border:1px solid #cfcabb;padding:0 2px}
table{border-collapse:collapse;margin:3mm 0;font-size:8.8pt;background:#f7f5ee}
tr{break-inside:avoid}
th,td{border:2px solid #141414;padding:1.2mm 1.8mm;vertical-align:top}
th{background:#141414;color:#e9e6dc;font-family:"DejaVu Sans",sans-serif;font-weight:700}
figure{margin:4mm 0;break-inside:avoid;text-align:center}
figure img{max-width:100%;border:3px solid #141414;box-shadow:2mm 2mm 0 #14141433;background:#f7f5ee}
figcaption,.cap{font:9pt/1.35 Typewriter,"DejaVu Sans Mono",monospace;margin-top:2mm;text-align:left}
figure:first-of-type img{border:4px solid #141414}
img{max-width:100%}
p>img{border:3px solid #141414;box-shadow:2mm 2mm 0 #14141433;display:block;margin:3mm auto}
p>img.cover{border:4px solid #141414}
table.pics{background:none;margin:4mm 0 0;width:100%}
table.pics td{border:none;text-align:center;vertical-align:top;padding:0 2mm}
table.pics img{max-height:118mm;border:3px solid #141414;box-shadow:2mm 2mm 0 #14141433}
table.page{width:100%;border-collapse:collapse;background:none;margin:0;font-size:inherit}
table.page>thead>tr>td,table.page>tfoot>tr>td,table.page>tbody>tr>td{border:none}
table.page>thead>tr>td,table.page>tfoot>tr>td{height:15mm;padding:0}
table.page>tbody>tr>td{padding:0 19mm}
hr{border:none;border-top:3px solid #141414}
.small{font-size:8pt;line-height:1.35;border-top:2px solid #141414;padding-top:2mm}
.small li{margin:0}
.lede{font:9pt/1.35 Typewriter,"DejaVu Sans Mono",monospace}
strong{font-weight:700}
"""

def main():
    frag = subprocess.run(["pandoc", "-f", "markdown+lists_without_preceding_blankline+implicit_figures",
                           "-t", "html5", MD], capture_output=True, text=True, check=True).stdout
    html = ("<!doctype html><html><head><meta charset='utf-8'><title>A Frankenstein Experience</title>"
            "<style>" + CSS + "</style></head><body><div class='bg'></div><div class='strip t'></div>"
            "<div class='strip b'></div><div class='edge l'></div><div class='edge r'></div>"
            "<table class='page'><thead><tr><td></td></tr></thead><tfoot><tr><td></td></tr></tfoot>"
            "<tbody><tr><td>" + frag + "</td></tr></tbody></table></body></html>")
    tmp = os.path.join(HERE, ".paper.html")
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(html)
    from playwright.sync_api import sync_playwright
    with sync_playwright() as p:
        exe = os.environ.get("CHROMIUM") or None
        b = p.chromium.launch(**({"executable_path": exe} if exe else {}))
        pg = b.new_page()
        pg.goto("file://" + tmp)
        pg.wait_for_timeout(600)
        pg.pdf(path=PDF, format="A4", print_background=True, prefer_css_page_size=True)
        b.close()
    os.remove(tmp)
    print("built", PDF)

if __name__ == "__main__":
    sys.exit(main())
