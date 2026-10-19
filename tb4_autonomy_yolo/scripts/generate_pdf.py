#!/usr/bin/env python3
"""Script chuyển đổi REPORT_LAB05.md sang REPORT_LAB05.pdf thông qua HTML và LibreOffice."""
import os
import re
import subprocess
from pathlib import Path

def convert_md_to_pdf():
    base_dir = Path(__file__).resolve().parent.parent
    report_dir = base_dir / 'report'
    md_path = report_dir / 'REPORT_LAB05.md'
    html_path = report_dir / 'REPORT_LAB05.html'
    pdf_path = report_dir / 'REPORT_LAB05.pdf'

    if not md_path.exists():
        print(f"Lỗi: Không tìm thấy {md_path}")
        return False

    with open(md_path, 'r', encoding='utf-8') as f:
        text = f.read()

    lines = text.split('\n')
    html_lines = []
    in_code = False
    in_table = False

    for line in lines:
        if line.startswith('```'):
            if in_code:
                html_lines.append('</pre>')
                in_code = False
            else:
                html_lines.append('<pre><code>')
                in_code = True
            continue
        if in_code:
            html_lines.append(line.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;'))
            continue

        # Tables
        if line.strip().startswith('|') and line.strip().endswith('|'):
            if not in_table:
                html_lines.append('<table>')
                in_table = True
            cells = [c.strip() for c in line.strip()[1:-1].split('|')]
            if all(re.match(r'^:?-+:?$', c) for c in cells):
                continue
            row_tag = 'th' if any(k in line for k in ['Mục', 'Thứ tự', 'Tên', 'Đối tượng', 'Giao thức']) else 'td'
            row_str = '<tr>' + ''.join(f'<{row_tag}>{c}</{row_tag}>' for c in cells) + '</tr>'
            html_lines.append(row_str)
            continue
        elif in_table:
            html_lines.append('</table>')
            in_table = False

        # Headings
        if line.startswith('# '):
            html_lines.append(f'<h1 class="title">{line[2:]}</h1>')
        elif line.startswith('## '):
            html_lines.append(f'<h2>{line[3:]}</h2>')
        elif line.startswith('### '):
            html_lines.append(f'<h3>{line[4:]}</h3>')
        elif line.startswith('#### '):
            html_lines.append(f'<h4>{line[5:]}</h4>')
        elif line.startswith('* ') or line.startswith('- '):
            l = line[2:]
            l = re.sub(r'\*\*(.*?)\*\*', r'<strong>\1</strong>', l)
            l = re.sub(r'`(.*?)`', r'<code>\1</code>', l)
            html_lines.append(f'<li>{l}</li>')
        elif line.strip() == '---':
            html_lines.append('<hr/>')
        elif line.strip():
            l = re.sub(r'\*\*(.*?)\*\*', r'<strong>\1</strong>', line)
            l = re.sub(r'`(.*?)`', r'<code>\1</code>', l)
            html_lines.append(f'<p>{l}</p>')

    if in_table:
        html_lines.append('</table>')

    html_content = f'''<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>Báo Cáo Lab 05 - TurtleBot 4 Autonomy & YOLO</title>
<style>
  @page {{
    size: A4;
    margin: 1.8cm 1.5cm;
  }}
  body {{
    font-family: 'DejaVu Sans', Arial, sans-serif;
    font-size: 10pt;
    line-height: 1.45;
    color: #1e293b;
  }}
  h1.title {{
    color: #0f172a;
    text-align: center;
    font-size: 16pt;
    margin-bottom: 4px;
    text-transform: uppercase;
    font-weight: bold;
    border-bottom: 2px solid #2563eb;
    padding-bottom: 8px;
  }}
  h2 {{
    color: #1e40af;
    font-size: 12pt;
    margin-top: 18px;
    margin-bottom: 6px;
    border-bottom: 1px solid #cbd5e1;
    padding-bottom: 3px;
  }}
  h3 {{
    color: #2563eb;
    font-size: 10.5pt;
    margin-top: 12px;
    margin-bottom: 4px;
  }}
  h4 {{
    color: #334155;
    font-size: 9.5pt;
    margin-top: 8px;
    margin-bottom: 3px;
  }}
  p {{
    margin: 4px 0;
  }}
  pre {{
    background: #f8fafc;
    border: 1px solid #e2e8f0;
    border-radius: 4px;
    padding: 6px 10px;
    font-family: 'DejaVu Sans Mono', monospace;
    font-size: 7.8pt;
    line-height: 1.25;
    color: #0f172a;
    margin: 6px 0;
  }}
  code {{
    font-family: 'DejaVu Sans Mono', monospace;
    background: #f1f5f9;
    padding: 1px 3px;
    border-radius: 3px;
    font-size: 8.5pt;
    color: #0284c7;
  }}
  table {{
    width: 100%;
    border-collapse: collapse;
    margin: 8px 0;
    font-size: 8.5pt;
  }}
  th, td {{
    border: 1px solid #cbd5e1;
    padding: 4px 8px;
    text-align: left;
  }}
  th {{
    background-color: #f1f5f9;
    color: #0f172a;
    font-weight: bold;
  }}
  tr:nth-child(even) {{
    background-color: #f8fafc;
  }}
  li {{
    margin-left: 18px;
    margin-bottom: 2px;
  }}
  hr {{
    border: none;
    border-top: 1px solid #e2e8f0;
    margin: 12px 0;
  }}
</style>
</head>
<body>
{''.join(html_lines)}
</body>
</html>'''

    with open(html_path, 'w', encoding='utf-8') as f:
        f.write(html_content)
    print(f"1. Đã sinh HTML: {html_path}")

    cmd = ['libreoffice', '--headless', '--convert-to', 'pdf', str(html_path), '--outdir', str(report_dir)]
    subprocess.run(cmd, check=True)
    if pdf_path.exists():
        print(f"2. Đã chuyển đổi thành công sang PDF: {pdf_path} (Kích thước: {pdf_path.stat().st_size} bytes)")
        return True
    return False

if __name__ == '__main__':
    convert_md_to_pdf()
