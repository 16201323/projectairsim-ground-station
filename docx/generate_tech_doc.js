const fs = require("fs");
const {
  Document, Packer, Paragraph, TextRun, Table, TableRow, TableCell,
  Header, Footer, AlignmentType, HeadingLevel, BorderStyle, WidthType,
  ShadingType, PageNumber, PageBreak, LevelFormat, TableOfContents,
} = require("docx");

const border = { style: BorderStyle.SINGLE, size: 1, color: "CCCCCC" };
const borders = { top: border, bottom: border, left: border, right: border };
const headerShading = { fill: "1B3A5C", type: ShadingType.CLEAR };
const altShading = { fill: "F0F4F8", type: ShadingType.CLEAR };
const cellMargins = { top: 60, bottom: 60, left: 100, right: 100 };

function hCell(text, width) {
  return new TableCell({
    borders, width: { size: width, type: WidthType.DXA },
    shading: headerShading, margins: cellMargins,
    children: [new Paragraph({ children: [new TextRun({ text, bold: true, color: "FFFFFF", font: "Microsoft YaHei", size: 20 })] })],
  });
}

function dCell(text, width, shading) {
  return new TableCell({
    borders, width: { size: width, type: WidthType.DXA },
    shading: shading || undefined, margins: cellMargins,
    children: [new Paragraph({ children: [new TextRun({ text, font: "Microsoft YaHei", size: 20 })] })],
  });
}

function makeRow(cells, isAlt) {
  return new TableRow({ children: cells.map((c, i) => dCell(c.text, c.width, isAlt ? altShading : undefined)) });
}

function makeTable(headers, rows, colWidths) {
  const tableWidth = colWidths.reduce((a, b) => a + b, 0);
  return new Table({
    width: { size: tableWidth, type: WidthType.DXA },
    columnWidths: colWidths,
    rows: [
      new TableRow({ children: headers.map((h, i) => hCell(h, colWidths[i])) }),
      ...rows.map((r, idx) => makeRow(r.map((text, i) => ({ text, width: colWidths[i] })), idx % 2 === 1)),
    ],
  });
}

function heading1(text) {
  return new Paragraph({ heading: HeadingLevel.HEADING_1, spacing: { before: 360, after: 200 }, children: [new TextRun({ text, bold: true, font: "Microsoft YaHei", size: 32 })] });
}
function heading2(text) {
  return new Paragraph({ heading: HeadingLevel.HEADING_2, spacing: { before: 280, after: 160 }, children: [new TextRun({ text, bold: true, font: "Microsoft YaHei", size: 28 })] });
}
function heading3(text) {
  return new Paragraph({ heading: HeadingLevel.HEADING_3, spacing: { before: 200, after: 120 }, children: [new TextRun({ text, bold: true, font: "Microsoft YaHei", size: 24 })] });
}
function para(text, opts = {}) {
  return new Paragraph({ spacing: { after: 120 }, children: [new TextRun({ text, font: "Microsoft YaHei", size: 20, ...opts })] });
}
function boldPara(label, text) {
  return new Paragraph({ spacing: { after: 100 }, children: [
    new TextRun({ text: label, font: "Microsoft YaHei", size: 20, bold: true }),
    new TextRun({ text, font: "Microsoft YaHei", size: 20 }),
  ]});
}
function codeBlock(text) {
  return new Paragraph({ spacing: { after: 80 }, indent: { left: 360 }, children: [new TextRun({ text, font: "Consolas", size: 18, color: "2E4057" })] });
}
function bulletItem(text) {
  return new Paragraph({ numbering: { reference: "bullets", level: 0 }, spacing: { after: 60 }, children: [new TextRun({ text, font: "Microsoft YaHei", size: 20 })] });
}

const doc = new Document({
  styles: {
    default: { document: { run: { font: "Microsoft YaHei", size: 20 } } },
    paragraphStyles: [
      { id: "Heading1", name: "Heading 1", basedOn: "Normal", next: "Normal", quickFormat: true, run: { size: 32, bold: true, font: "Microsoft YaHei" }, paragraph: { spacing: { before: 360, after: 200 }, outlineLevel: 0 } },
      { id: "Heading2", name: "Heading 2", basedOn: "Normal", next: "Normal", quickFormat: true, run: { size: 28, bold: true, font: "Microsoft YaHei" }, paragraph: { spacing: { before: 280, after: 160 }, outlineLevel: 1 } },
      { id: "Heading3", name: "Heading 3", basedOn: "Normal", next: "Normal", quickFormat: true, run: { size: 24, bold: true, font: "Microsoft YaHei" }, paragraph: { spacing: { before: 200, after: 120 }, outlineLevel: 2 } },
    ],
  },
  numbering: {
    config: [
      { reference: "bullets", levels: [{ level: 0, format: LevelFormat.BULLET, text: "\u2022", alignment: AlignmentType.LEFT, style: { paragraph: { indent: { left: 720, hanging: 360 } } } }] },
    ],
  },
  sections: [{
    properties: {
      page: { size: { width: 11906, height: 16838 }, margin: { top: 1440, right: 1200, bottom: 1440, left: 1200 } },
    },
    headers: {
      default: new Header({ children: [new Paragraph({ alignment: AlignmentType.RIGHT, children: [new TextRun({ text: "ProjectAirSim \u5730\u9762\u7ad9\u6280\u672f\u6587\u6863", font: "Microsoft YaHei", size: 16, color: "888888", italics: true })] })] }),
    },
    footers: {
      default: new Footer({ children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [
        new TextRun({ text: "\u7B2C ", font: "Microsoft YaHei", size: 16, color: "888888" }),
        new TextRun({ children: [PageNumber.CURRENT], font: "Microsoft YaHei", size: 16, color: "888888" }),
        new TextRun({ text: " \u9875", font: "Microsoft YaHei", size: 16, color: "888888" }),
      ] })] }),
    },
    children: [
      new Paragraph({ spacing: { after: 200 }, children: [new TextRun({ text: "ProjectAirSim \u5730\u9762\u7ad9\u63a7\u5236\u7cfb\u7edf", font: "Microsoft YaHei", size: 44, bold: true, color: "1B3A5C" })] }),
      new Paragraph({ spacing: { after: 100 }, children: [new TextRun({ text: "\u6280\u672f\u6587\u6863 v1.0", font: "Microsoft YaHei", size: 28, color: "4A7FB5" })] }),
      para(""),
      makeTable(["\u5c5e\u6027", "\u5185\u5bb9"], [
        ["\u9879\u76ee\u540d\u79f0", "ProjectAirSim \u5730\u9762\u7ad9\u63a7\u5236\u7cfb\u7edf"],
        ["\u6846\u67b6", "PyQt6 + ProjectAirSim Python SDK"],
        ["\u4eff\u771f\u5f15\u64ce", "Unreal Engine (DynamicCity \u9884\u7f16\u8bd1\u73af\u5883)"],
        ["\u7f16\u7a0b\u8bed\u8a00", "Python 3.10+"],
        ["\u6587\u6863\u65e5\u671f", "2026-05-08"],
        ["\u7528\u9014", "\u65b0\u9879\u76ee\u8fc1\u79fb\u53c2\u8003"],
      ], [2400, 6700]),
      new Paragraph({ children: [new PageBreak()] }),
      new TableOfContents("\u76ee\u5f55", { hyperlink: true, headingStyleRange: "1-3" }),
      new Paragraph({ children: [new PageBreak()] }),

      // ===== 1 =====
      heading1("1  \u9879\u76ee\u6982\u8ff0"),
      heading2("1.1  \u6838\u5fc3\u529f\u80fd"),
      bulletItem("\u6df1\u8272\u9713\u8679\u79d1\u5e7b\u98ce\u683cUI\u754c\u9762\uff08Dark Neon\u4e3b\u9898\uff09"),
      bulletItem("\u652f\u6301\u4e09\u79cd\u65e0\u4eba\u673a\u578b\u53f7\uff1a\u56db\u65cb\u7ffc / \u516d\u65cb\u7ffc / \u503e\u659c\u65cb\u7ffc(VTOL)"),
      bulletItem("\u53cc\u63a7\u5236\u6a21\u5f0f\uff1a\u952e\u76d8\u624b\u52a8\u63a7\u5236 + UDP\u81ea\u52a8\u63a7\u5236"),
      bulletItem("\u5b9e\u65f6\u76f8\u673a\u89c6\u9891\u6d41\u663e\u793a\uff08\u524d\u89c6/\u4e0b\u89c6/\u8ffd\u8e2a/\u53cc\u76ee\uff09"),
      bulletItem("\u591a\u4f20\u611f\u5668\u6570\u636e\u5b9e\u65f6\u76d1\u63a7\uff08IMU/GPS/\u9ad8\u5ea6\u8868/\u5927\u6c14\u673a/\u96f7\u8fbe\uff09"),
      bulletItem("\u5b8c\u6574\u98de\u884c\u63a7\u5236\u6d41\uff1a\u542f\u52a8\u2192\u98de\u884c\u2192\u7740\u9646\u2192\u9000\u51fa\uff08\u652f\u6301\u91cd\u590d\u542f\u52a8\uff09"),
      bulletItem("UDP\u8d85\u65f6\u81ea\u52a8\u60ac\u505c\u4fdd\u62a4"),
      bulletItem("\u98de\u884c\u6570\u636e\u5f55\u5236\u4e0e\u4fdd\u5b58"),

      heading2("1.2  \u9879\u76ee\u7ed3\u6784"),
      codeBlock("\u251c\u2500\u2500 main.py                  # \u4e3b\u7a0b\u5e8f\u5165\u53e3"),
      codeBlock("\u251c\u2500\u2500 core/"),
      codeBlock("\u2502   \u251c\u2500\u2500 constants.py         # \u5168\u5c40\u5e38\u91cf"),
      codeBlock("\u2502   \u251c\u2500\u2500 control_thread.py   # \u98de\u884c\u63a7\u5236\u7ebf\u7a0b"),
      codeBlock("\u2502   \u251c\u2500\u2500 config_manager.py   # \u4eff\u771f\u914d\u7f6e\u7ba1\u7406"),
      codeBlock("\u2502   \u251c\u2500\u2500 data_recorder.py    # \u6570\u636e\u8bb0\u5f55\u5668"),
      codeBlock("\u2502   \u2514\u2500\u2500 udp_manager.py      # UDP\u901a\u4fe1\u7ba1\u7406"),
      codeBlock("\u251c\u2500\u2500 sensors/"),
      codeBlock("\u2502   \u251c\u2500\u2500 base.py              # \u4f20\u611f\u5668\u57fa\u7c7b"),
      codeBlock("\u2502   \u251c\u2500\u2500 factory.py           # \u4f20\u611f\u5668\u5de5\u5382"),
      codeBlock("\u2502   \u251c\u2500\u2500 manager.py           # \u4f20\u611f\u5668\u7ba1\u7406\u5668"),
      codeBlock("\u2502   \u251c\u2500\u2500 imu.py / gps.py / altimeter.py / atmosphere.py / radar.py / camera.py / stereo_camera.py"),
      codeBlock("\u251c\u2500\u2500 ui/"),
      codeBlock("\u2502   \u251c\u2500\u2500 sensor_panel.py / video_widget.py / widgets.py"),
      codeBlock("\u251c\u2500\u2500 sim_config/"),
      codeBlock("\u2502   \u251c\u2500\u2500 robot_quadrotor_adv.jsonc / robot_hexarotor_adv.jsonc / robot_quadtiltrotor_adv.jsonc"),
      codeBlock("\u2502   \u2514\u2500\u2500 scene_adv_drone.jsonc"),
      codeBlock("\u2514\u2500\u2500 requirements.txt"),

      // ===== 2 =====
      new Paragraph({ children: [new PageBreak()] }),
      heading1("2  \u754c\u9762\u5e03\u5c40"),
      heading2("2.1  \u6574\u4f53\u5e03\u5c40"),
      para("\u754c\u9762\u91c7\u7528\u4e0a-\u4e0b\u4e8c\u5c42\u5e03\u5c40\uff0c\u4e0a\u5c42\u6807\u9898\u680f\uff0c\u4e0b\u5c42\u4e3b\u5de5\u4f5c\u533a\uff1a"),
      codeBlock("\u250c\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2510"),
      codeBlock("\u2502  \u25c6 AIRSIM GROUND STATION  [\u542f\u52a8][\u7740\u9646][\u9000\u51fa]  [\u8fde\u63a5][\u98de\u884c]  \u65f6\u949f  \u2502"),
      codeBlock("\u251c\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u252c\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2524"),
      codeBlock("\u2502  \u5de6\u4fa7\u9762\u677f  \u2502              \u89c6\u9891\u663e\u793a\u533a\u57df              \u2502"),
      codeBlock("\u2502  \u98de\u673a\u7c7b\u578b  \u2502  [\u524d\u89c6/\u4e0b\u89c6] [\u62cd\u7167] [VTOL]            \u2502"),
      codeBlock("\u2502  \u63a7\u5236\u6a21\u5f0f  \u2502  \u250c\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2510  \u2502"),
      codeBlock("\u2502  \u98de\u884c\u901f\u5ea6  \u2502  \u2502         \u76f8\u673a\u89c6\u9891                     \u2502  \u2502"),
      codeBlock("\u2502  UDP\u53c2\u6570   \u2502  \u2514\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2518  \u2502"),
      codeBlock("\u2502  \u4f20\u611f\u5668    \u251c\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2524"),
      codeBlock("\u2502            \u2502              \u8fd0\u884c\u65e5\u5fd7              \u2502"),
      codeBlock("\u2514\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2534\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2518"),

      heading2("2.2  \u5404\u533a\u57df\u8bf4\u660e"),
      makeTable(["\u533a\u57df", "\u5185\u5bb9"], [
        ["\u6807\u9898\u680f(70px)", "\u25c6\u6807\u9898 + \u542f\u52a8/\u7740\u9646/\u9000\u51fa\u6309\u94ae + \u8fde\u63a5/\u98de\u884c\u72b6\u6001\u6307\u793a\u5668 + \u65f6\u949f"],
        ["\u5de6\u4fa7\u9762\u677f(240px)", "\u98de\u673a\u7c7b\u578b\u5355\u9009 + \u63a7\u5236\u6a21\u5f0f\u5355\u9009 + \u901f\u5ea6\u8fdb\u5ea6\u6761 + UDP\u53c2\u6570(12\u9879) + \u4f20\u611f\u5668\u6570\u636e\u9762\u677f"],
        ["\u53f3\u4e0a\u89c6\u9891\u533a", "\u524d\u89c6/\u4e0b\u89c6\u5207\u6362 + \u62cd\u7167 + VTOL\u5207\u6362 + VideoWidget"],
        ["\u53f3\u4e0b\u65e5\u5fd7\u533a", "\u53ea\u8bfbQTextEdit\uff0c\u6700\u591a1000\u6761\uff0cINFO/WARNING/ERROR\u4e09\u8272"],
      ], [2800, 6300]),

      heading2("2.3  \u7a97\u53e3\u5c3a\u5bf8\u5e38\u91cf"),
      makeTable(["\u5e38\u91cf", "\u503c", "\u8bf4\u660e"], [
        ["WINDOW_WIDTH", "1400", "\u7a97\u53e3\u9ed8\u8ba4\u5bbd\u5ea6"],
        ["WINDOW_HEIGHT", "900", "\u7a97\u53e3\u9ed8\u8ba4\u9ad8\u5ea6"],
        ["LEFT_PANEL_WIDTH", "240", "\u5de6\u4fa7\u9762\u677f\u5bbd\u5ea6"],
        ["BOTTOM_PANEL_HEIGHT", "280", "\u5e95\u90e8\u65e5\u5fd7\u9ad8\u5ea6"],
        ["CAMERA_WIDTH/HEIGHT", "640/360", "\u8ffd\u8e2a\u76f8\u673a\u5f55\u50cf\u5206\u8fa8\u7387"],
        ["VIDEO_FPS", "20", "\u5f55\u50cf\u5e27\u7387"],
      ], [3000, 2000, 4100]),

      // ===== 3 =====
      new Paragraph({ children: [new PageBreak()] }),
      heading1("3  \u4f20\u611f\u5668\u53c2\u6570\u914d\u7f6e"),

      heading2("3.1  \u4f20\u611f\u5668\u6e05\u5355"),
      makeTable(["ID", "\u7c7b\u578b", "\u8bf4\u660e"], [
        ["IMU1", "imu", "\u60ef\u6027\u6d4b\u91cf\u5355\u5143"],
        ["GPS", "gps", "\u5168\u7403\u5b9a\u4f4d"],
        ["RadioAltimeter", "distance-sensor", "\u65e0\u7ebf\u7535\u9ad8\u5ea6\u8868 0~500m"],
        ["LaserAltimeter", "distance-sensor", "\u6fc0\u5149\u9ad8\u5ea6\u8868 0~300m"],
        ["UltrasonicAltimeter", "distance-sensor", "\u8d85\u58f0\u6ce2\u9ad8\u5ea6\u8868 0~10m"],
        ["Barometer", "barometer", "\u6c14\u538b\u8ba1"],
        ["Airspeed", "airspeed", "\u7a7a\u901f\u4f20\u611f\u5668"],
        ["lidar1", "lidar", "3D\u70b9\u4e91 16\u901a\u9053 360\u00b0"],
        ["Radar1", "radar", "\u6beb\u7c73\u6ce2\u96f7\u8fbe \u00b160\u00b0"],
        ["Chase", "camera", "\u8ffd\u8e2a\u76f8\u673a"],
        ["DownCamera", "camera", "\u4e0b\u89c6\u6df1\u5ea6\u76f8\u673a"],
        ["StereoLeft", "camera", "\u53cc\u76ee\u5de6(\u5bbd120\u00b0)"],
        ["StereoRight", "camera", "\u53cc\u76ee\u53f3(\u7a8430\u00b0)"],
      ], [2800, 2400, 3900]),

      heading2("3.2  \u76f8\u673a\u53c2\u6570"),
      makeTable(["\u53c2\u6570", "Chase", "DownCamera", "StereoLeft", "StereoRight"], [
        ["\u5206\u8fa8\u7387", "3840\u00d72160", "3840\u00d72160", "3840\u00d72160", "3840\u00d72160"],
        ["H-FOV", "120\u00b0", "120\u00b0", "120\u00b0", "30\u00b0"],
        ["capture-interval", "0.033s", "0.033s", "0.033s", "0.033s"],
        ["image-type", "0(RGB)", "0+1(Depth)+3(Seg)", "0(RGB)", "0(RGB)"],
        ["\u4f4d\u7f6e xyz", "-4 0 -1.2", "0 0 0", "0.25 -0.06 -0.10", "0.25 0.06 -0.10"],
        ["\u59ff\u6001 rpy-deg", "0 -8 0", "0 -90 0", "0 0 0", "0 0 0"],
        ["\u4e91\u53f0", "\u4e09\u8f74\u81ea\u7531", "\u65e0", "\u65e0", "\u65e0"],
      ], [2200, 1800, 2200, 1600, 1300]),
      para(""),
      boldPara("\u6027\u80fd\u4f18\u5316\u5efa\u8bae\uff1a", "\u5f53\u524d4\u8def3840\u00d72160\u662f\u5361\u987f\u4e3b\u56e0\u3002\u65b0\u9879\u76ee\u5efa\u8bae640\u00d7360\u62161280\u00d7720\u3002"),

      heading2("3.3  LiDAR\u53c2\u6570"),
      makeTable(["\u53c2\u6570", "\u503c", "\u8bf4\u660e"], [
        ["lidar-type", "generic_cylindrical", "\u901a\u7528\u5706\u67f1\u5f0f"],
        ["number-of-channels", "16", "\u5782\u76f4\u901a\u9053\uff08\u4ece32\u964d\u81f316\uff09"],
        ["range", "200m", "\u6d4b\u91cf\u8303\u56f4\uff08\u4ece300\u964d\u81f3200\uff09"],
        ["points-per-second", "50000", "\u6bcf\u79d2\u70b9\u6570\uff08\u4ece100k\u964d\u81f350k\uff09"],
        ["horizontal-rotation-frequency", "10Hz", "\u6c34\u5e73\u65cb\u8f6c\u9891\u7387"],
        ["horizontal-fov", "0\u00b0~360\u00b0", "\u5168\u5411\u626b\u63cf"],
        ["vertical-fov", "+15\u00b0~-25\u00b0", "\u5411\u4e0a15\u00b0\u5411\u4e0b25\u00b0"],
        ["disable-self-hits", "true", "\u4e0d\u68c0\u6d4b\u81ea\u8eab"],
        ["origin xyz", "0 0 0.2", "\u673a\u8eab\u4e2d\u5fc3\u4e0a\u65b920cm"],
        ["report-point-cloud", "true", "\u8ba2\u9605\u83b7\u53d6\u70b9\u4e91"],
      ], [3200, 2200, 3700]),
      para(""),
      boldPara("LiDAR\u5b50\u7c7b\u578b\uff1a", ""),
      bulletItem("generic_cylindrical\uff1a\u5706\u67f1\u5f0f\uff0c\u6027\u80fd\u7a33\u5b9a\u63a8\u8350"),
      bulletItem("generic_rosette\uff1a\u73ab\u7470\u82b1\u5f0f\uff0c\u9700vertical-rotation-frequency\u53c2\u6570"),
      bulletItem("depth_lidar\uff1a\u57fa\u4e8e\u6df1\u5ea6\u56fe\uff0c\u70b9\u4e91\u5bc6\u5ea6\u4e0e\u5206\u8fa8\u7387\u76f8\u5173"),
      bulletItem("gpu_cylindrical\uff1aGPU\u52a0\u901f\uff0c\u4f1a\u4e0e\u573a\u666f\u6e32\u67d3\u7ade\u4e89GPU\u5bfc\u81f4\u5361\u6b7b\uff0c\u4e0d\u63a8\u8350"),

      heading2("3.4  \u96f7\u8fbe\u53c2\u6570\uff08\u6a21\u4effUCM241\uff09"),
      makeTable(["\u53c2\u6570", "\u503c", "\u8bf4\u660e"], [
        ["range-max", "150m", "\u6d4b\u8ddd\u8303\u56f4"],
        ["range-resolution", "0.3m", "\u8ddd\u79bb\u5206\u8fa8\u7387"],
        ["azimuth-resolution", "3.0\u00b0", "\u65b9\u4f4d\u89d2\u5206\u8fa8\u7387"],
        ["elevation-resolution", "4.0\u00b0", "\u4ef0\u89d2\u5206\u8fa8\u7387"],
        ["velocity-resolution", "0.3m/s", "\u901f\u5ea6\u5206\u8fa8\u7387"],
        ["horizontal-fov", "-60\u00b0~+60\u00b0", "\u65b9\u4f4d\u89d2\u8303\u56f4"],
        ["vertical-fov", "-30\u00b0~+10\u00b0", "\u4ef0\u89d2\u8303\u56f4"],
        ["number-of-targets", "64", "\u6700\u5927\u76ee\u6807\u6570"],
      ], [3200, 2200, 3700]),

      heading2("3.5  \u9ad8\u5ea6\u8868\u53c2\u6570"),
      makeTable(["\u4f20\u611f\u5668", "\u8303\u56f4", "\u7cbe\u5ea6", "\u4f4d\u7f6e xyz", "rpy-deg"], [
        ["RadioAltimeter", "0.5~500m", "\u00b10.5m", "0 0 0.05", "0 -90 0"],
        ["LaserAltimeter", "0.2~300m", "\u00b10.1m", "0.1 0 0.05", "0 -90 0"],
        ["UltrasonicAltimeter", "0.02~10m", "\u00b10.02m", "-0.1 0 0.05", "0 -90 0"],
      ], [2000, 1600, 1400, 2000, 2100]),

      heading2("3.6  IMU\u53c2\u6570"),
      makeTable(["\u53c2\u6570", "\u52a0\u901f\u8ba1", "\u9640\u87ba\u4eea"], [
        ["\u968f\u673a\u6e38\u8d70", "0.0123 m/s\u00b2/\u221aHz", "0.0123 rad/s/\u221aHz"],
        ["\u65f6\u95f4\u5e38\u6570\u03c4", "800s", "500s"],
        ["\u504f\u5dee\u7a33\u5b9a\u6027", "2e-5 m/s\u00b2", "1e-6 rad/s"],
        ["\u5f00\u673a\u504f\u5dee", "0 0 0", "0 0 0"],
      ], [2800, 3200, 3100]),

      heading2("3.7  \u573a\u666f\u914d\u7f6e"),
      makeTable(["\u53c2\u6570", "\u503c"], [
        ["scene-type", "UnrealNative"],
        ["home-geo-point", "\u7eac\u5ea629.2687 \u7ecf\u5ea6117.1784 \u6d77\u62d450m"],
        ["clock type", "steppable"],
        ["step-ns", "3000000 (3ms)"],
        ["real-time-update-rate", "3000000 (1:1)"],
        ["pause-on-start", "false"],
      ], [3200, 5900]),

      // ===== 4 =====
      new Paragraph({ children: [new PageBreak()] }),
      heading1("4  \u5355\u4f4d\u8f6c\u6362\u4e0eBug\u4fee\u590d"),

      heading2("4.1  \u5355\u4f4d\u8f6c\u6362\u603b\u89c8"),
      para("\u4ee5\u4e0b\u662f\u5df2\u786e\u8ba4\u7684\u5355\u4f4d\u8f6c\u6362\u89c4\u5219\uff0c\u65b0\u9879\u76ee\u5fc5\u987b\u4e25\u683c\u9075\u5b88\uff1a"),
      makeTable(["\u6570\u636e\u7c7b\u578b", "C++\u7aef\u5355\u4f4d", "Python\u7aef\u5904\u7406", "\u9700\u8f6c\u6362?"], [
        ["distance-sensor", "\u5398\u7c73(UE\u6807\u51c6)", "\u00f7100\u8f6c\u7c73", "\u662f(C++ bug)"],
        ["radar range", "\u7c73(\u5df2\u505aToMeters)", "\u76f4\u63a5\u7528", "\u5426"],
        ["radar azimuth/elevation", "\u5ea6(\u975e\u5f27\u5ea6!)", "\u76f4\u63a5\u7528", "\u5426(\u52ff\u8c03math.degrees!)"],
        ["IMU\u56db\u5143\u6570\u2192\u6b27\u62c9\u89d2", "\u56db\u5143\u6570", "atan2+degrees()", "\u662f(\u5f27\u5ea6\u2192\u5ea6)"],
        ["GPS lat/lon/alt", "\u5ea6/\u7c73", "\u76f4\u63a5\u7528", "\u5426"],
        ["\u4e91\u53f0 rpy-deg", "\u5ea6", "radians()\u8f6c\u5f27\u5ea6", "\u662f(\u5ea6\u2192\u5f27\u5ea6)"],
        ["\u7ecf\u7eac\u5ea6\u2192NED", "\u5ea6", "geo_to_ned_coordinates()", "\u662f(SDK\u5185\u90e8)"],
        ["yaw_rate", "\u5ea6/s", "radians()\u8f6c\u5f27\u5ea6/s", "\u662f(\u5ea6\u2192\u5f27\u5ea6)"],
      ], [2200, 2200, 2600, 2100]),

      heading2("4.2  Bug\u4fee\u590d\u8bb0\u5f55"),

      heading3("4.2.1  distance-sensor\u5358\u4f4d\u9519\u8bef(\u5398\u7c73\u2192\u7c73)"),
      boldPara("\u95ee\u9898: ", "C++\u7aefFHitResult.Distance\u5355\u4f4d\u4e3a\u5398\u7c73\uff0c\u4f20\u5165DistanceSensorMessage\u65f6\u672a\u505aToMeters()\u8f6c\u6362\u3002"),
      boldPara("\u4fee\u590d: ", "distance_m = distance_cm / 100.0"),
      boldPara("\u6e90\u7801: ", "UnrealDistanceSensor.cpp:161 \u2192 Distance=HitInfo.Distance(\u5398\u7c73); :65 \u2192 \u672a\u8f6c\u6362"),

      heading3("4.2.2  \u96f7\u8fbe\u89d2\u5ea6\u53cc\u91cd\u8f6c\u6362"),
      boldPara("\u95ee\u9898: ", "C++\u7aefazimuth/elevation\u5df2\u662f\u5ea6\u6570\u503c\uff0cPython\u7aef\u8bef\u7528math.degrees()\u5bfc\u81f4\u53cc\u91cd\u8f6c\u6362\u3002"),
      boldPara("\u4fee\u590d: ", "\u76f4\u63a5\u4f7f\u7528\u539f\u59cb\u5ea6\u6570\u503c\u3002\u4f8b: azimuth=-37.0 \u2192 math.degrees(-37.0)=-2119.9\u00b0(\u9519\u8bef!)"),

      heading3("4.2.3  \u96f7\u8fbe\u5b57\u6bb5\u540d\u9519\u8bef"),
      boldPara("\u95ee\u9898: ", "C++\u7aefMSGPACK key\u662f\"radar_detections\"\uff0c\u4e4b\u524d\u8bef\u7528\"detections\"\u3002"),
      boldPara("\u4fee\u590d: ", "radar_data.get(\"radar_detections\", [])"),

      heading3("4.2.4  LiDAR\u914d\u7f6e\u8fde\u5b57\u7b26\u95ee\u9898"),
      boldPara("\u95ee\u9898: ", "\"generic-cylindrical\"\u5e94\u4e3a\"generic_cylindrical\"\uff0c\u5bfc\u81f4\u573a\u666f\u52a0\u8f7d\u5931\u8d25\u3002"),
      boldPara("\u4fee\u590d: ", "JSONC\u4e2d\u4f7f\u7528\u4e0b\u5212\u7ebf\u800c\u975e\u8fde\u5b57\u7b26\u3002"),

      heading3("4.2.5  LiDAR\u56de\u8c03\u53cc\u91cd\u8c03\u7528"),
      boldPara("\u95ee\u9898: ", "__call__\u4e2d\u4e24\u6b21\u8c03\u7528_should_update_ui()\uff0c\u7b2c\u4e8c\u6b21\u6c38\u8fdc\u8fd4\u56defALSE\u3002"),
      boldPara("\u4fee\u590d: ", "\u5408\u5e76\u4e3a\u4e00\u6b21\u8c03\u7528\uff0c\u590d\u7528\u7ed3\u679c\u3002"),

      heading3("4.2.6  NumPy 2.0\u4e0d\u517c\u5bb9"),
      boldPara("\u95ee\u9898: ", "ndarray.ptp()\u5728NumPy 2.0\u4e2d\u88ab\u79fb\u9664\u3002"),
      boldPara("\u4fee\u590d: ", "pts.ptp(axis=0) \u2192 pts.max(axis=0) - pts.min(axis=0)"),

      heading3("4.2.7  Win+D\u5bfc\u81f4\u65e0\u4eba\u673a\u6301\u7eed\u53f3\u79fb"),
      boldPara("\u95ee\u9898: ", "\u7a97\u53e3\u5931\u7126\u65f6pynput\u5ffd\u7565key release\uff0c\u6309\u952e\u72b6\u6001\u6b8b\u7559\u3002"),
      boldPara("\u4fee\u590d: ", "\u4fee\u6539pynput\u91ca\u653e\u903b\u8f91+\u7a97\u53e3\u5931\u6d3b\u68c0\u6d4b+\u4fee\u9970\u952e\u5c4f\u853d\u3002"),

      heading3("4.2.8  generic_rosette\u7f3a\u53c2\u6570"),
      boldPara("\u95ee\u9898: ", "generic_rosette\u9700\u8981vertical-rotation-frequency\u53c2\u6570\u3002"),
      boldPara("\u4fee\u590d: ", "\u6dfb\u52a0vertical-rotation-frequency: 2\u3002"),

      heading3("4.2.9  gpu_cylindrical\u5361\u6b7b"),
      boldPara("\u95ee\u9898: ", "GPU\u8d44\u6e90\u7ade\u4e89\u5bfc\u81f4\u573a\u666f\u53d1\u767d\u5361\u6b7b\u3002"),
      boldPara("\u4fee\u590d: ", "\u56de\u9000generic_cylindrical\uff0c\u964d\u4f4e\u70b9\u6570\u548c\u901a\u9053\u6570\u3002"),

      heading3("4.2.10  \u70b9\u4e91\u5de6\u53f3\u4ea4\u66ff\u6d88\u5931"),
      boldPara("\u95ee\u9898: ", "\u7a84FOV\u4e0b\u6bcf\u5e27\u53ea\u663e\u793a\u6247\u5f62\u533a\u57df\uff0c\u4e0d\u7d2f\u79ef\u5386\u53f2\u5e27\u3002"),
      boldPara("\u4fee\u590d: ", "\u5b9e\u73b0\u70b9\u4e91\u7d2f\u79ef\u7f13\u51b2\u533a\uff0c\u4fdd\u7559\u6700\u8fd130\u5e27\u6570\u636e\u3002"),

      // ===== 5 =====
      new Paragraph({ children: [new PageBreak()] }),
      heading1("5  \u6027\u80fd\u4f18\u5316\u5efa\u8bae"),

      heading2("5.1  \u5361\u987f\u6839\u56e0\u5206\u6790"),
      bulletItem("\u76f8\u673a\u5206\u8fa8\u7387\u8fc7\u9ad8\uff1a4\u8def3840\u00d72160\uff0c\u6bcf\u5e27\u7ea62.7MB\uff0c4\u76f8\u673a\u00d720fps=80\u6b21/\u79d2\u8de8\u7ebf\u7a0b\u4f20\u8f93"),
      bulletItem("\u4f20\u611f\u5668UI\u66f4\u65b0\u65e0\u8282\u6d41\uff1a\u6bcf\u6b21\u56de\u8c03\u90fd\u89e6\u53d1UI\u5237\u65b0\uff0c\u672a\u505a\u6279\u91cf\u5408\u5e76"),
      bulletItem("\u952e\u76d8\u63a7\u5236\u5faa\u73af\u4e0e\u6e32\u67d3\u7ade\u4e89\uff1a10ms\u63a7\u5236\u5faa\u73af+\u5e27\u62c9\u53d6\u5b9a\u65f6\u5668\u540c\u65f6\u8fd0\u884c"),

      heading2("5.2  \u65b0\u9879\u76ee\u4f18\u5316\u65b9\u6848"),
      makeTable(["\u4f18\u5316\u70b9", "\u5f53\u524d", "\u5efa\u8bae", "\u9884\u671f\u6548\u679c"], [
        ["\u76f8\u673a\u5206\u8fa8\u7387", "3840\u00d72160", "640\u00d7360\u62161280\u00d7720", "\u5e27\u4f20\u8f93\u91cf\u964d\u81f31/9~1/36"],
        ["\u5e27\u62c9\u53d6\u9891\u7387", "15fps(66ms)", "10fps(100ms)", "\u51cf\u5c11UI\u91cd\u7ed8\u6b21\u6570"],
        ["\u4f20\u611f\u5668UI\u66f4\u65b0", "\u6bcf\u6b21\u56de\u8c03\u89e6\u53d1", "\u7edf\u4e00\u5b9a\u65f6\u5668100ms\u6279\u91cf\u66f4\u65b0", "\u907f\u514d\u9ad8\u9891\u5237\u65b0"],
        ["\u63a7\u5236\u5faa\u73af", "10ms\u8f6e\u8be2", "20ms\u8f6e\u8be2", "\u51cf\u5c11CPU\u5360\u7528"],
        ["\u65e5\u5fd7\u663e\u793a", "\u6bcf\u6b21append", "\u7d2f\u79ef\u540e\u5b9a\u65f6\u5237\u65b0", "\u51cf\u5c11QTextEdit\u91cd\u7ed8"],
      ], [2000, 2200, 2400, 2500]),

      // ===== 6 =====
      heading1("6  \u952e\u76d8\u63a7\u5236\u6620\u5c04"),
      makeTable(["\u6309\u952e", "\u529f\u80fd"], [
        ["W/S", "\u524d\u8fdb/\u540e\u9000"],
        ["A/D", "\u5de6\u79fb/\u53f3\u79fb"],
        ["\u65b9\u5411\u952e\u2191\u2193", "\u4e0a\u5347/\u4e0b\u964d"],
        ["\u65b9\u5411\u952e\u2190\u2192", "\u5de6\u8f6c/\u53f3\u8f6c"],
        ["Space", "\u7d27\u6025\u60ac\u505c"],
        ["T", "\u8d77\u98de"],
        ["L", "\u964d\u843d"],
        ["+/-", "\u52a0\u51cf\u901f\u5ea6"],
        ["Win/Ctrl/Alt+\u4efb\u610f\u952e", "\u5c4f\u853d\uff0c\u4e0d\u89e6\u53d1\u63a7\u5236"],
      ], [3000, 6100]),

      // ===== 7 =====
      heading1("7  UDP\u901a\u4fe1\u534f\u8bae"),
      heading2("7.1  \u914d\u7f6e\u53c2\u6570"),
      makeTable(["\u53c2\u6570", "\u503c"], [
        ["UDP_DEFAULT_IP", "192.168.1.5"],
        ["UDP_DEFAULT_PORT", "15610"],
        ["UDP_MULTICAST_ADDR", "224.0.0.25"],
        ["UDP_BUFFER_SIZE", "1024"],
        ["UDP_RECV_TIMEOUT", "0.1s"],
      ], [3200, 5900]),

      heading2("7.2  \u63a7\u5236\u6307\u4ee4\u683c\u5f0f"),
      para("JSON\u683c\u5f0f\uff0c\u652f\u6301velocity\u548cposition\u4e24\u79cd\u6a21\u5f0f\uff1a"),
      codeBlock('{"velocity": {"vx":3.0, "vy":0.0, "vz":0.0}, "yaw_rate": 0.0}'),
      codeBlock('{"position": {"x":10.0, "y":5.0, "z":-5.0}}'),
      para("velocity\u4f18\u5148\u7ea7\u9ad8\u4e8eposition\u3002\u5355\u4f4d\uff1avx/vy/vz(m/s)\uff0cyaw_rate(\u00b0/s)\uff0cx/y/z(NED\u5750\u6807\u7c73)\u3002"),

      heading2("7.3  UDP\u53c2\u6570\u9762\u677f\u663e\u793a\u5b57\u6bb5"),
      makeTable(["\u5b57\u6bb5", "\u5355\u4f4d", "\u8bf4\u660e"], [
        ["lon/lat", "\u00b0", "\u7ecf\u5ea6/\u7eac\u5ea6"],
        ["alt/height", "m", "\u76f8\u5bf9/\u7edd\u5bf9\u9ad8\u5ea6"],
        ["theta/phi/psi", "\u00b0", "\u4fef\u4ef0/\u6eda\u8f6c/\u504f\u822a\u89d2"],
        ["Vt/Vi", "m/s", "\u771f\u7a7a\u901f/\u6307\u793a\u7a7a\u901f"],
        ["vn/ve", "m/s", "\u5317\u5411/\u4e1c\u5411\u901f\u5ea6"],
        ["Hdot/Vd", "m/s", "\u5347\u964d\u901f\u5ea6/\u5730\u901f"],
      ], [2400, 1200, 5500]),

      // ===== 8 =====
      heading1("8  \u989c\u8272\u65b9\u6848"),
      makeTable(["\u5e38\u91cf", "\u8272\u503c", "\u7528\u9014"], [
        ["COLOR_BG_MAIN", "#0a0e17", "\u4e3b\u80cc\u666f(\u6df1\u9ed1\u84dd)"],
        ["COLOR_BG_PANEL", "#141b2d", "\u9762\u677f\u80cc\u666f"],
        ["COLOR_BG_PANEL_LIGHT", "#1a2340", "\u6d45\u9762\u677f/\u6309\u94ae"],
        ["COLOR_BORDER", "#1e3a5f", "\u8fb9\u6846"],
        ["COLOR_NEON_CYAN", "#00d4ff", "\u4e3b\u5f3a\u8c03\u8272"],
        ["COLOR_NEON_GREEN", "#00ff88", "\u6210\u529f/\u98de\u884c"],
        ["COLOR_NEON_YELLOW", "#ffd700", "\u8b66\u544a/\u7740\u9646"],
        ["COLOR_NEON_RED", "#ff4444", "\u9519\u8bef/\u65ad\u5f00"],
        ["COLOR_TEXT_MAIN", "#e0e6ed", "\u4e3b\u6587\u5b57"],
        ["COLOR_TEXT_SECOND", "#8892a0", "\u8f85\u52a9\u6587\u5b57"],
      ], [3000, 2000, 4100]),

      // ===== 9 =====
      heading1("9  \u4f20\u611f\u5668\u67b6\u6784\u8bbe\u8ba1"),
      heading2("9.1  \u7c7b\u56fe"),
      codeBlock("SensorCallback (\u62bd\u8c61\u57fa\u7c7b)"),
      codeBlock("  \u251c\u2500\u2500 CameraCallback       # \u76f8\u673a"),
      codeBlock("  \u251c\u2500\u2500 StereoCameraCallback  # \u53cc\u76ee\u76f8\u673a"),
      codeBlock("  \u251c\u2500\u2500 IMUCallback          # IMU"),
      codeBlock("  \u251c\u2500\u2500 GPSCallback          # GPS"),
      codeBlock("  \u251c\u2500\u2500 AltimeterCallback    # \u9ad8\u5ea6\u8868\u57fa\u7c7b"),
      codeBlock("  \u2502   \u251c\u2500\u2500 RadioAltimeterCallback"),
      codeBlock("  \u2502   \u251c\u2500\u2500 LaserAltimeterCallback"),
      codeBlock("  \u2502   \u2514\u2500\u2500 UltrasonicAltimeterCallback"),
      codeBlock("  \u251c\u2500\u2500 AtmosphereCallback   # \u5927\u6c14\u673a"),
      codeBlock("  \u2514\u2500\u2500 RadarCallback        # \u96f7\u8fbe"),

      heading2("9.2  \u6838\u5fc3\u8bbe\u8ba1"),
      bulletItem("SensorType\u679a\u4e3e\uff1a\u7edf\u4e00\u7ba1\u7406\u6240\u6709\u4f20\u611f\u5668\u7c7b\u578b"),
      bulletItem("SensorData\u6570\u636e\u7c7b\uff1a\u7edf\u4e00\u5c01\u88c5(sensor_type, sensor_name, timestamp, payload)"),
      bulletItem("SensorFactory\u5de5\u5382\uff1a\u6839\u636eSensorType\u81ea\u52a8\u521b\u5efa\u56de\u8c03\u5b9e\u4f8b"),
      bulletItem("SensorManager\u7ba1\u7406\u5668\uff1a\u7edf\u4e00\u8ba2\u9605\u3001\u6570\u636e\u5206\u53d1\u3001\u751f\u547d\u5468\u671f\u7ba1\u7406"),
      bulletItem("UI\u8282\u6d41\u673a\u5236\uff1a_should_update_ui()\u63a7\u5236\u6700\u5c0f\u66f4\u65b0\u95f4\u96940.2s"),

      heading2("9.3  \u4f20\u611f\u5668\u7c7b\u578b\u63a8\u65ad\u4f18\u5148\u7ea7"),
      para("1. ID_TYPE_MAP\u7cbe\u786e\u5339\u914d\uff08\u5982RadioAltimeter\u2192RADIO_ALTIMETER\uff09"),
      para("2. \u8bdd\u9898key\u63a8\u65ad\uff08\u5982\u542b\"imu_kinematics\"\u2192IMU\uff09"),
      para("3. \u4f20\u611f\u5668ID\u5173\u952e\u8bcd\u63a8\u65ad\uff08\u5982\u542b\"altimeter\"\u2192\u9ad8\u5ea6\u8868\uff09"),
    ],
  }],
});

Packer.toBuffer(doc).then(buffer => {
  fs.writeFileSync("d:\\code\\projectairsim\\ProjectAirSim\\client\\python\\mine\\docx\\ProjectAirSim_\u5730\u9762\u7ad9\u6280\u672f\u6587\u6863.docx", buffer);
  console.log("Document created successfully!");
});
