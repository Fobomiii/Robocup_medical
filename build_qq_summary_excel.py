from __future__ import annotations

from datetime import datetime, timezone
from html import escape
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile
import xml.etree.ElementTree as ET


OUTPUT_PATH = Path(__file__).with_name("2026社团巡礼节_详细信息汇总.xlsx")


def cell_ref(column: int, row: int) -> str:
    letters = ""
    current = column
    while current:
        current, remainder = divmod(current - 1, 26)
        letters = chr(65 + remainder) + letters
    return f"{letters}{row}"


def make_row(values, style=4, overrides=None, height=None):
    return {
        "values": list(values),
        "style": style,
        "overrides": overrides or {},
        "height": height,
    }


def worksheet_xml(sheet):
    rows = sheet["rows"]
    max_columns = max(len(row["values"]) for row in rows)
    max_rows = len(rows)
    parts = [
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>',
        '<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" '
        'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">',
        f'<dimension ref="A1:{cell_ref(max_columns, max_rows)}"/>',
        '<sheetViews><sheetView showGridLines="0" workbookViewId="0">',
    ]
    freeze_row = sheet.get("freeze_row")
    if freeze_row:
        top_left = f"A{freeze_row + 1}"
        parts.append(
            f'<pane ySplit="{freeze_row}" topLeftCell="{top_left}" '
            'activePane="bottomLeft" state="frozen"/>'
        )
    parts.extend([
        '</sheetView></sheetViews>',
        '<sheetFormatPr defaultRowHeight="18"/>',
        '<cols>',
    ])
    for index, width in enumerate(sheet["widths"], start=1):
        parts.append(f'<col min="{index}" max="{index}" width="{width}" customWidth="1"/>')
    parts.extend(['</cols>', '<sheetData>'])
    for row_number, row in enumerate(rows, start=1):
        row_attributes = [f'r="{row_number}"']
        if row["height"]:
            row_attributes.extend([f'ht="{row["height"]}"', 'customHeight="1"'])
        parts.append(f'<row {" ".join(row_attributes)}>')
        for column_number, value in enumerate(row["values"], start=1):
            if value is None:
                continue
            reference = cell_ref(column_number, row_number)
            style = row["overrides"].get(column_number, row["style"])
            text = escape(str(value))
            preserve = ' xml:space="preserve"' if str(value).strip() != str(value) else ""
            parts.append(
                f'<c r="{reference}" s="{style}" t="inlineStr">'
                f'<is><t{preserve}>{text}</t></is></c>'
            )
        parts.append('</row>')
    parts.append('</sheetData>')
    auto_filter = sheet.get("auto_filter")
    if auto_filter:
        parts.append(f'<autoFilter ref="{auto_filter}"/>')
    merges = sheet.get("merges", [])
    if merges:
        parts.append(f'<mergeCells count="{len(merges)}">')
        parts.extend(f'<mergeCell ref="{reference}"/>' for reference in merges)
        parts.append('</mergeCells>')
    hyperlinks = sheet.get("hyperlinks", {})
    if hyperlinks:
        parts.append('<hyperlinks>')
        for relationship_id, reference in enumerate(hyperlinks, start=1):
            parts.append(f'<hyperlink ref="{reference}" r:id="rId{relationship_id}"/>')
        parts.append('</hyperlinks>')
    parts.append(
        '<pageMargins left="0.3" right="0.3" top="0.5" bottom="0.5" '
        'header="0.2" footer="0.2"/>'
    )
    parts.append('</worksheet>')
    return "".join(parts)


def worksheet_relationships_xml(hyperlinks):
    parts = [
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>',
        '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">',
    ]
    for relationship_id, target in enumerate(hyperlinks.values(), start=1):
        parts.append(
            f'<Relationship Id="rId{relationship_id}" '
            'Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/hyperlink" '
            f'Target="{escape(target, quote=True)}" TargetMode="External"/>'
        )
    parts.append('</Relationships>')
    return "".join(parts)


def workbook_xml(sheets):
    sheet_entries = "".join(
        f'<sheet name="{escape(sheet["name"], quote=True)}" sheetId="{index}" r:id="rId{index + 1}"/>'
        for index, sheet in enumerate(sheets, start=1)
    )
    return (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<workbook xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" '
        'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">'
        '<workbookPr/><bookViews><workbookView xWindow="0" yWindow="0" '
        'windowWidth="24000" windowHeight="12000"/></bookViews>'
        f'<sheets>{sheet_entries}</sheets>'
        '<calcPr calcId="191029"/></workbook>'
    )


def workbook_relationships_xml(sheet_count):
    relationships = [
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>',
        '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">',
        '<Relationship Id="rId1" '
        'Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles" '
        'Target="styles.xml"/>',
    ]
    for index in range(1, sheet_count + 1):
        relationships.append(
            f'<Relationship Id="rId{index + 1}" '
            'Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" '
            f'Target="worksheets/sheet{index}.xml"/>'
        )
    relationships.append('</Relationships>')
    return "".join(relationships)


def content_types_xml(sheet_count):
    overrides = "".join(
        f'<Override PartName="/xl/worksheets/sheet{index}.xml" '
        'ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>'
        for index in range(1, sheet_count + 1)
    )
    return (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
        '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
        '<Default Extension="xml" ContentType="application/xml"/>'
        '<Override PartName="/xl/workbook.xml" '
        'ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml"/>'
        '<Override PartName="/xl/styles.xml" '
        'ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.styles+xml"/>'
        '<Override PartName="/docProps/core.xml" '
        'ContentType="application/vnd.openxmlformats-package.core-properties+xml"/>'
        '<Override PartName="/docProps/app.xml" '
        'ContentType="application/vnd.openxmlformats-officedocument.extended-properties+xml"/>'
        f'{overrides}</Types>'
    )


def styles_xml():
    return '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<styleSheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main">
  <fonts count="7">
    <font><sz val="11"/><name val="Microsoft YaHei"/><family val="2"/></font>
    <font><b/><color rgb="FFFFFFFF"/><sz val="16"/><name val="Microsoft YaHei"/><family val="2"/></font>
    <font><b/><color rgb="FFFFFFFF"/><sz val="11"/><name val="Microsoft YaHei"/><family val="2"/></font>
    <font><sz val="11"/><name val="Microsoft YaHei"/><family val="2"/></font>
    <font><b/><color rgb="FF7F6000"/><sz val="11"/><name val="Microsoft YaHei"/><family val="2"/></font>
    <font><u/><color rgb="FF0563C1"/><sz val="11"/><name val="Microsoft YaHei"/><family val="2"/></font>
    <font><i/><color rgb="FF666666"/><sz val="10"/><name val="Microsoft YaHei"/><family val="2"/></font>
  </fonts>
  <fills count="9">
    <fill><patternFill patternType="none"/></fill>
    <fill><patternFill patternType="gray125"/></fill>
    <fill><patternFill patternType="solid"><fgColor rgb="FF1F4E78"/><bgColor indexed="64"/></patternFill></fill>
    <fill><patternFill patternType="solid"><fgColor rgb="FF5B9BD5"/><bgColor indexed="64"/></patternFill></fill>
    <fill><patternFill patternType="solid"><fgColor rgb="FFFFF2CC"/><bgColor indexed="64"/></patternFill></fill>
    <fill><patternFill patternType="solid"><fgColor rgb="FFE2F0D9"/><bgColor indexed="64"/></patternFill></fill>
    <fill><patternFill patternType="solid"><fgColor rgb="FFFCE4D6"/><bgColor indexed="64"/></patternFill></fill>
    <fill><patternFill patternType="solid"><fgColor rgb="FFE7E6E6"/><bgColor indexed="64"/></patternFill></fill>
    <fill><patternFill patternType="solid"><fgColor rgb="FFF4CCCC"/><bgColor indexed="64"/></patternFill></fill>
  </fills>
  <borders count="2">
    <border><left/><right/><top/><bottom/><diagonal/></border>
    <border><left style="thin"><color rgb="FFD9E2F3"/></left><right style="thin"><color rgb="FFD9E2F3"/></right><top style="thin"><color rgb="FFD9E2F3"/></top><bottom style="thin"><color rgb="FFD9E2F3"/></bottom><diagonal/></border>
  </borders>
  <cellStyleXfs count="1"><xf numFmtId="0" fontId="0" fillId="0" borderId="0"/></cellStyleXfs>
  <cellXfs count="12">
    <xf numFmtId="0" fontId="0" fillId="0" borderId="0" xfId="0"/>
    <xf numFmtId="0" fontId="1" fillId="2" borderId="1" xfId="0" applyFont="1" applyFill="1" applyBorder="1" applyAlignment="1"><alignment horizontal="center" vertical="center"/></xf>
    <xf numFmtId="0" fontId="2" fillId="3" borderId="1" xfId="0" applyFont="1" applyFill="1" applyBorder="1" applyAlignment="1"><alignment vertical="center"/></xf>
    <xf numFmtId="0" fontId="2" fillId="2" borderId="1" xfId="0" applyFont="1" applyFill="1" applyBorder="1" applyAlignment="1"><alignment horizontal="center" vertical="center" wrapText="1"/></xf>
    <xf numFmtId="0" fontId="3" fillId="0" borderId="1" xfId="0" applyFont="1" applyBorder="1" applyAlignment="1"><alignment vertical="top" wrapText="1"/></xf>
    <xf numFmtId="0" fontId="4" fillId="4" borderId="1" xfId="0" applyFont="1" applyFill="1" applyBorder="1" applyAlignment="1"><alignment vertical="top" wrapText="1"/></xf>
    <xf numFmtId="0" fontId="3" fillId="5" borderId="1" xfId="0" applyFont="1" applyFill="1" applyBorder="1" applyAlignment="1"><alignment vertical="top" wrapText="1"/></xf>
    <xf numFmtId="0" fontId="4" fillId="6" borderId="1" xfId="0" applyFont="1" applyFill="1" applyBorder="1" applyAlignment="1"><alignment vertical="top" wrapText="1"/></xf>
    <xf numFmtId="0" fontId="6" fillId="7" borderId="1" xfId="0" applyFont="1" applyFill="1" applyBorder="1" applyAlignment="1"><alignment vertical="top" wrapText="1"/></xf>
    <xf numFmtId="0" fontId="5" fillId="0" borderId="1" xfId="0" applyFont="1" applyBorder="1" applyAlignment="1"><alignment vertical="top" wrapText="1"/></xf>
    <xf numFmtId="0" fontId="4" fillId="8" borderId="1" xfId="0" applyFont="1" applyFill="1" applyBorder="1" applyAlignment="1"><alignment vertical="top" wrapText="1"/></xf>
    <xf numFmtId="0" fontId="3" fillId="7" borderId="1" xfId="0" applyFont="1" applyFill="1" applyBorder="1" applyAlignment="1"><alignment vertical="top" wrapText="1"/></xf>
  </cellXfs>
  <cellStyles count="1"><cellStyle name="Normal" xfId="0" builtinId="0"/></cellStyles>
  <dxfs count="0"/>
  <tableStyles count="0" defaultTableStyle="TableStyleMedium2" defaultPivotStyle="PivotStyleLight16"/>
</styleSheet>'''


def app_xml(sheets):
    titles = "".join(f'<vt:lpstr>{escape(sheet["name"])}</vt:lpstr>' for sheet in sheets)
    return (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<Properties xmlns="http://schemas.openxmlformats.org/officeDocument/2006/extended-properties" '
        'xmlns:vt="http://schemas.openxmlformats.org/officeDocument/2006/docPropsVTypes">'
        '<Application>Microsoft Excel Compatible</Application><DocSecurity>0</DocSecurity>'
        '<ScaleCrop>false</ScaleCrop><HeadingPairs><vt:vector size="2" baseType="variant">'
        '<vt:variant><vt:lpstr>Worksheets</vt:lpstr></vt:variant>'
        f'<vt:variant><vt:i4>{len(sheets)}</vt:i4></vt:variant></vt:vector></HeadingPairs>'
        f'<TitlesOfParts><vt:vector size="{len(sheets)}" baseType="lpstr">{titles}</vt:vector></TitlesOfParts>'
        '<Company></Company><LinksUpToDate>false</LinksUpToDate><SharedDoc>false</SharedDoc>'
        '<HyperlinksChanged>false</HyperlinksChanged><AppVersion>16.0300</AppVersion></Properties>'
    )


def core_xml():
    timestamp = datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")
    return (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<cp:coreProperties xmlns:cp="http://schemas.openxmlformats.org/package/2006/metadata/core-properties" '
        'xmlns:dc="http://purl.org/dc/elements/1.1/" '
        'xmlns:dcterms="http://purl.org/dc/terms/" '
        'xmlns:dcmitype="http://purl.org/dc/dcmitype/" '
        'xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">'
        '<dc:title>2026社团巡礼节详细信息汇总</dc:title><dc:creator>Codex</dc:creator>'
        '<cp:keywords>QQ聊天记录,社团巡礼节,党服,材料截止</cp:keywords>'
        '<dc:description>根据本地QQ群聊中可核验消息整理</dc:description>'
        f'<dcterms:created xsi:type="dcterms:W3CDTF">{timestamp}</dcterms:created>'
        f'<dcterms:modified xsi:type="dcterms:W3CDTF">{timestamp}</dcterms:modified>'
        '</cp:coreProperties>'
    )


def build_sheets():
    overview_rows = [
        make_row(["2026社团巡礼节社团工作群｜详细信息汇总"], style=1, height=34),
        make_row(["群聊", "2026社团巡礼节社团工作群", "群号", "1127728867", "本地消息范围", "78条", "整理日期", "2026-10-09"], style=11, height=28),
        make_row([None], style=0, height=8),
        make_row(["关键结论"], style=2, height=24),
        make_row(["党服人数限制", "群聊中未说明人数限制或人数上限；不能据此认定为不限人数。明确“不限人数”的是志愿汇。"], style=5, overrides={1: 3}, height=42),
        make_row(["党服时长", "按实际工作时长计算。"], style=4, overrides={1: 3}, height=28),
        make_row(["党服表单", "https://f.wps.cn/g/UX5L4XkK/"], style=4, overrides={1: 3, 2: 9}, height=28),
        make_row(["党服表单截止", "2026-10-09 22:00（群消息称“今晚22:00”）"], style=10, overrides={1: 3}, height=28),
        make_row(["未填党服表单", "默认计为志愿时长。"], style=7, overrides={1: 3}, height=28),
        make_row(["建议", "若需要确认党服可申报人数，应直接向群内负责人或社团部确认；当前聊天依据不足以给出人数数字。"], style=5, overrides={1: 3}, height=40),
        make_row([None], style=0, height=8),
        make_row(["重点截止"], style=2, height=24),
        make_row(["事项", "截止时间", "提交/办理方式", "状态与提醒"], style=3, height=28),
        make_row(["易拉宝材料", "2026-09-29 23:00", "发送至 1954138055@qq.com；发送后私信确认", "所有参与社团均须提交；9月30日再次催交，未交者需尽快补交。"], style=4, overrides={2: 10}, height=42),
        make_row(["接电情况摸排", "2026-10-02 18:00", "按群内摸排要求提交", "此为群内最新截止。"], style=4, overrides={2: 10}, height=32),
        make_row(["集体请假名单", "2026-10-09 22:00", "https://www.kdocs.cn/l/cmXZkzE6iO6B", "需要集体请假的人员填写。"], style=4, overrides={2: 10, 3: 9}, height=32),
        make_row(["党服时长表单", "2026-10-09 22:00", "https://f.wps.cn/g/UX5L4XkK/", "需要党服时长者填写；未填默认志愿时长。"], style=4, overrides={2: 10, 3: 9}, height=36),
        make_row([None], style=0, height=8),
        make_row(["分析口径"], style=2, height=24),
        make_row(["“群内未说明”不等于“没有限制”。本表只写入本地聊天中能核验的明确要求；聊天依据页为内容摘要，并非逐字导出。"], style=11, height=38),
        make_row(["时间口径", "所有时间均按 QQ 消息中显示的日期与时间记录，未做时区换算。"], style=8, overrides={1: 3}, height=30),
    ]

    people_rows = [
        make_row(["人员限制与时长规则"], style=1, height=34),
        make_row(["注意：党服未给出人数上限；志愿汇明确不限人数；德育名额是另一套限制。"], style=5, height=34),
        make_row(["类别", "项目", "人数限制", "能否叠加", "时长/次数", "负责方", "表单/编码", "截止", "说明"], style=3, height=32),
        make_row(["党服", "党服时长申报", "群内未说明", "群内未说明", "按实际工作时长", "需要党服时长的人员填写", "https://f.wps.cn/g/UX5L4XkK/", "2026-10-09 22:00", "未填写表单默认计为志愿时长；如需人数数字，须向负责人确认。"], style=4, overrides={3: 5, 7: 9, 8: 10}, height=54),
        make_row(["志愿汇", "志愿时长", "不限人数", "不适用", "按实际工作时长", "社团部统一申请", "活动当天发出编码", "未说明", "签到、签退范围仅限活动现场；一般无需社团自行申请。"], style=4, overrides={3: 6}, height=48),
        make_row(["德育", "摆摊人员", "≤3人", "三类人员不可叠加", "E2类 2次", "按群通知执行", "不适用", "未说明", "与节目表演、宣讲共同受每社团总计≤20人的约束。"], style=4, height=42),
        make_row(["德育", "节目表演人员", "≤15人", "三类人员不可叠加", "E2类 2次", "按群通知执行", "不适用", "未说明", "与摆摊、宣讲共同受每社团总计≤20人的约束。"], style=4, height=42),
        make_row(["德育", "宣讲人员", "≤2人", "三类人员不可叠加", "E2类 2次", "按群通知执行", "不适用", "未说明", "与摆摊、节目表演共同受每社团总计≤20人的约束。"], style=4, height=42),
        make_row(["德育", "每社团总计", "≤20人", "三类人员不可叠加", "E2类 2次", "按群通知执行", "不适用", "未说明", "总计限制与各分类上限同时适用。"], style=4, overrides={3: 7}, height=42),
    ]

    material_rows = [
        make_row(["易拉宝材料清单"], style=1, height=34),
        make_row(["统一截止：2026-09-29 23:00；邮箱：1954138055@qq.com；邮件发送后需私信确认。"], style=7, height=34),
        make_row(["序号", "材料", "数量", "规格/内容", "文件命名", "打包名称", "提交方式", "截止", "备注"], style=3, height=32),
        make_row(["1", "活动特色照片", "4张", "高清原图", "社团名称+活动1、社团名称+活动2、社团名称+活动3、社团名称+活动4", "社团名称+易拉宝材料", "1954138055@qq.com", "2026-09-29 23:00", "邮件发送后私信确认。"], style=4, overrides={7: 9, 8: 10}, height=56),
        make_row(["2", "社团 Logo", "1张", "高清、无水印、透明底或白底", "社团名称+logo", "社团名称+易拉宝材料", "1954138055@qq.com", "2026-09-29 23:00", "邮件发送后私信确认。"], style=4, overrides={7: 9, 8: 10}, height=48),
        make_row(["3", "社团介绍", "1份", "约300字，Word文档", "社团名称+介绍", "社团名称+易拉宝材料", "1954138055@qq.com", "2026-09-29 23:00", "邮件发送后私信确认。"], style=4, overrides={7: 9, 8: 10}, height=48),
        make_row(["补充", "提交范围", "全部参与社团", "所有参与社团均须提交", "按上述规则", "社团名称+易拉宝材料", "同上", "原定截止已过", "9月30日群内再次催交；未提交者需尽快补交。"], style=7, height=48),
    ]

    deadline_rows = [
        make_row(["截止时间与待办"], style=1, height=34),
        make_row(["按截止时间排序；红色为群内明确的截止时刻。"], style=8, height=28),
        make_row(["优先级", "事项", "截止时间", "需准备/填写", "提交渠道", "提交后动作", "责任提示", "当前口径"], style=3, height=32),
        make_row(["已到期", "易拉宝材料", "2026-09-29 23:00", "4张活动原图、1张Logo、约300字Word介绍，按规则命名并打包", "1954138055@qq.com", "私信确认", "所有参与社团", "原定截止后群内催交；未交应尽快补交。"], style=4, overrides={1: 7, 3: 10, 5: 9}, height=58),
        make_row(["已到期", "接电情况摸排", "2026-10-02 18:00", "接电需求情况", "群内摸排渠道", "未说明", "有接电需求的社团", "群内最新截止为当日18:00。"], style=4, overrides={1: 7, 3: 10}, height=42),
        make_row(["当日截止", "集体请假名单", "2026-10-09 22:00", "需要集体请假的人员名单", "https://www.kdocs.cn/l/cmXZkzE6iO6B", "未说明", "需要请假的人员/社团", "群消息称“今晚22:00”。"], style=4, overrides={1: 5, 3: 10, 5: 9}, height=46),
        make_row(["当日截止", "党服时长表单", "2026-10-09 22:00", "需要党服时长的人员信息", "https://f.wps.cn/g/UX5L4XkK/", "未说明", "需要党服时长的人员", "未填默认志愿时长；人数上限未说明。"], style=4, overrides={1: 5, 3: 10, 5: 9}, height=46),
        make_row(["活动日", "社团巡礼节", "2026-10-11", "按活动安排到场、布场、签到签退", "铜盘校区", "活动后按时收场", "所有参与人员", "详细时段见“活动安排”工作表。"], style=4, overrides={1: 6}, height=42),
    ]

    activity_rows = [
        make_row(["活动安排与现场保障"], style=1, height=34),
        make_row(["活动日期：2026-10-11｜地点：铜盘校区"], style=6, height=30),
        make_row(["日期", "时间", "事项", "地点", "要求/说明", "责任提示"], style=3, height=32),
        make_row(["2026-10-11", "12:30–16:00", "节目彩排", "铜盘校区", "按通知时段参加彩排。", "有节目表演的社团"], style=4, height=34),
        make_row(["2026-10-11", "14:30前", "完成布场", "铜盘校区", "摊位布置须在14:30前完成。", "参展社团"], style=4, overrides={2: 5}, height=34),
        make_row(["2026-10-11", "15:00–18:30", "社团摊位", "铜盘校区", "开展摊位活动。", "参展社团"], style=4, height=34),
        make_row(["2026-10-11", "16:30–19:00", "舞台表演", "铜盘校区", "按节目安排参加演出。", "演出社团"], style=4, height=34),
        make_row(["2026-10-11", "18:30–19:30", "摊位收场", "铜盘校区", "按时完成撤场和整理。", "参展社团"], style=4, height=34),
        make_row(["2026-10-11", "活动当天", "志愿汇签到/签退", "仅限活动现场", "编码活动当天发出；签到签退范围仅限现场。", "需要志愿时长的人员"], style=4, height=42),
        make_row(["2026-10-11", "活动当天", "摊位配置", "铜盘校区", "每个摊位由两个单元拼成。", "参展社团"], style=4, height=34),
        make_row(["2026-10-11", "活动当天", "桌子", "铜盘校区", "桌子由华为提供，无需自行携带。", "参展社团"], style=4, height=34),
        make_row(["2026-10-11", "往返", "交通", "自行安排", "不统一安排大巴，各社团自行解决交通。", "各社团"], style=4, overrides={5: 7}, height=38),
        make_row(["2026-10-11", "活动期间", "餐饮", "铜盘校区", "无餐券。", "各社团自行安排"], style=4, overrides={5: 7}, height=34),
        make_row(["活动前", "一般情况", "策划与志愿申请", "不适用", "一般无需社团自行写策划或申请志愿时长，由社团部处理。", "社团部"], style=4, height=42),
    ]

    evidence_rows = [
        make_row(["聊天依据（相关消息摘要）"], style=1, height=34),
        make_row(["以下为相关消息的结构化摘要，并非逐字聊天导出；用于说明结论来源。"], style=8, height=32),
        make_row(["时间", "发送人", "Seq", "主题", "内容摘要（非逐字转录）", "支持的结论"], style=3, height=32),
        make_row(["2026-09-29 10:49:07", "辞尘.", "501", "德育/志愿汇人数", "德育：摆摊≤3人、节目表演≤15人、宣讲≤2人，三类不可叠加，每社团总计≤20人，E2类2次；志愿汇不限人数，由社团部统一申请。", "人员限制"], style=4, height=64),
        make_row(["2026-09-30 11:43:43", "Twinkle", "507", "易拉宝材料", "发布4张活动特色高清原图、1张高清无水印Logo、约300字Word介绍的命名、打包、邮箱、私信确认及9月29日23:00截止要求。", "材料清单/截止"], style=4, height=62),
        make_row(["2026-10-02 11:17:43", "-iiiiiv", "513", "接电摸排", "接电情况摸排的最新截止为当日18:00。", "接电截止"], style=4, height=38),
        make_row(["2026-10-08 14:24:26", "辞尘.", "518", "活动安排", "发布彩排、布场、社团摊位、舞台表演和摊位收场的完整时间表。", "活动时间"], style=4, height=44),
        make_row(["2026-10-09 11:36:26", "seagull", "536", "集体请假", "发布集体请假名单相关通知。", "请假任务"], style=4, height=34),
        make_row(["2026-10-09 11:36:41", "seagull", "537", "集体请假", "发布/补充集体请假表链接：https://www.kdocs.cn/l/cmXZkzE6iO6B", "请假表链接"], style=4, height=40),
        make_row(["2026-10-09 12:00:49", "seagull", "539", "请假截止", "集体请假名单截止今晚22:00。", "请假截止"], style=4, height=34),
        make_row(["2026-10-09 12:25:00", "seagull", "541", "党服表单", "发布党服时长表单：https://f.wps.cn/g/UX5L4XkK/", "党服表单"], style=4, height=40),
        make_row(["2026-10-09 12:25:17", "seagull", "542", "党服截止/默认规则", "党服表单截止今晚22:00；未填写默认计为志愿时长。", "党服截止与默认规则"], style=4, height=42),
        make_row(["2026-10-09 12:31:38", "ovO！", "543", "时长询问", "询问党服与志愿时长如何计算。", "问题背景"], style=4, height=34),
        make_row(["2026-10-09 12:39:27", "seagull", "545", "时长答复", "答复：按工作时长。", "党服/志愿时长"], style=4, height=34),
        make_row(["2026-10-09 17:51:46", "祈未遥", "548", "志愿人数询问", "询问志愿人数是否有限制；结合此前通知，志愿汇明确不限人数。", "志愿汇人数"], style=4, height=42),
        make_row(["结论性缺口", "—", "—", "党服人数", "相关消息没有给出党服人数上限，也没有明确写“党服不限人数”。", "因此只能标记为“群内未说明”，不能填具体数字。"], style=5, height=46),
    ]

    return [
        {
            "name": "总览",
            "rows": overview_rows,
            "widths": [20, 30, 25, 28, 25, 14, 18, 24],
            "freeze_row": 2,
            "merges": ["A1:H1", "A4:H4", "B5:H5", "B6:H6", "B7:H7", "B8:H8", "B9:H9", "B10:H10", "A12:H12", "A19:H19", "A20:H20", "B21:H21"],
            "hyperlinks": {
                "B7": "https://f.wps.cn/g/UX5L4XkK/",
                "C16": "https://www.kdocs.cn/l/cmXZkzE6iO6B",
                "C17": "https://f.wps.cn/g/UX5L4XkK/",
            },
        },
        {
            "name": "人员限制",
            "rows": people_rows,
            "widths": [12, 18, 18, 22, 18, 24, 30, 22, 42],
            "freeze_row": 3,
            "merges": ["A1:I1", "A2:I2"],
            "auto_filter": "A3:I9",
            "hyperlinks": {"G4": "https://f.wps.cn/g/UX5L4XkK/"},
        },
        {
            "name": "材料清单",
            "rows": material_rows,
            "widths": [9, 18, 12, 30, 42, 28, 25, 22, 35],
            "freeze_row": 3,
            "merges": ["A1:I1", "A2:I2"],
            "auto_filter": "A3:I7",
            "hyperlinks": {
                "G4": "mailto:1954138055@qq.com",
                "G5": "mailto:1954138055@qq.com",
                "G6": "mailto:1954138055@qq.com",
            },
        },
        {
            "name": "截止与任务",
            "rows": deadline_rows,
            "widths": [14, 20, 23, 48, 36, 18, 24, 40],
            "freeze_row": 3,
            "merges": ["A1:H1", "A2:H2"],
            "auto_filter": "A3:H8",
            "hyperlinks": {
                "E4": "mailto:1954138055@qq.com",
                "E6": "https://www.kdocs.cn/l/cmXZkzE6iO6B",
                "E7": "https://f.wps.cn/g/UX5L4XkK/",
            },
        },
        {
            "name": "活动安排",
            "rows": activity_rows,
            "widths": [18, 18, 24, 22, 48, 28],
            "freeze_row": 3,
            "merges": ["A1:F1", "A2:F2"],
            "auto_filter": "A3:F14",
            "hyperlinks": {},
        },
        {
            "name": "聊天依据",
            "rows": evidence_rows,
            "widths": [24, 16, 10, 22, 78, 30],
            "freeze_row": 3,
            "merges": ["A1:F1", "A2:F2"],
            "auto_filter": "A3:F16",
            "hyperlinks": {},
        },
    ]


def write_workbook():
    sheets = build_sheets()
    root_relationships = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
        '<Relationship Id="rId1" '
        'Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" '
        'Target="xl/workbook.xml"/>'
        '<Relationship Id="rId2" '
        'Type="http://schemas.openxmlformats.org/package/2006/relationships/metadata/core-properties" '
        'Target="docProps/core.xml"/>'
        '<Relationship Id="rId3" '
        'Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/extended-properties" '
        'Target="docProps/app.xml"/>'
        '</Relationships>'
    )
    with ZipFile(OUTPUT_PATH, "w", ZIP_DEFLATED, compresslevel=9) as archive:
        archive.writestr("[Content_Types].xml", content_types_xml(len(sheets)))
        archive.writestr("_rels/.rels", root_relationships)
        archive.writestr("docProps/app.xml", app_xml(sheets))
        archive.writestr("docProps/core.xml", core_xml())
        archive.writestr("xl/workbook.xml", workbook_xml(sheets))
        archive.writestr("xl/_rels/workbook.xml.rels", workbook_relationships_xml(len(sheets)))
        archive.writestr("xl/styles.xml", styles_xml())
        for index, sheet in enumerate(sheets, start=1):
            archive.writestr(f"xl/worksheets/sheet{index}.xml", worksheet_xml(sheet))
            if sheet["hyperlinks"]:
                archive.writestr(
                    f"xl/worksheets/_rels/sheet{index}.xml.rels",
                    worksheet_relationships_xml(sheet["hyperlinks"]),
                )
    with ZipFile(OUTPUT_PATH, "r") as archive:
        damaged = archive.testzip()
        if damaged:
            raise RuntimeError(f"ZIP校验失败：{damaged}")
        for name in archive.namelist():
            if name.endswith(".xml") or name.endswith(".rels"):
                ET.fromstring(archive.read(name))
    print(f"已生成：{OUTPUT_PATH}")
    print(f"文件大小：{OUTPUT_PATH.stat().st_size} bytes")
    print(f"工作表：{', '.join(sheet['name'] for sheet in sheets)}")


if __name__ == "__main__":
    write_workbook()
