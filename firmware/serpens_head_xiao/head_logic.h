// 頭の XIAO の純粋なロジック（ハードウェアに触れない。行の解釈・S 行の組み立て・LED のデューティ）。
// `serpens/hw/head_io.py` の `parse_*` / `format_*` の写し。HARDWARE_UNVERIFIED。**C++ としては実行していない**（ホストにコンパイラが無い。ESP32 向けのコンパイルのみ）。
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

enum HlKind { HL_NONE = 0, HL_PING, HL_EYE, HL_MODE, HL_LED };

struct HlCommand {
  HlKind kind;
  int a, b, c, d;       // EYE: r, g, b, brightness / MODE: a = mode / LED: a = 左, b = 右（0..255）
};

// 1 行（LF なし）を解釈する。不正な行は HL_NONE（黙って捨てる。Python と同じ規約）。
// 受け付ける形: "P" / "E,r,g,b,brightness" / "M,mode" / "L,left,right"（**L は PROPOSED**: 斜め LED の試験用。Python 側に呼び出しは無い）
static inline bool hlParseInts(const char* s, int* out, int n) {
  for (int i = 0; i < n; i++) {
    if (*s == '\0') return false;
    bool neg = false;
    if (*s == '-') { neg = true; s++; }
    if (*s < '0' || *s > '9') return false;
    long v = 0;
    while (*s >= '0' && *s <= '9') { v = v * 10 + (*s - '0'); if (v > 100000) return false; s++; }
    out[i] = (int)(neg ? -v : v);
    if (i < n - 1) { if (*s != ',') return false; s++; }
  }
  return *s == '\0';                       // 余りがあれば不正
}

static inline HlCommand hlParseLine(const char* line) {
  HlCommand cmd = {HL_NONE, 0, 0, 0, 0};
  if (line == NULL || line[0] == '\0') return cmd;
  if (line[0] == 'P' && line[1] == '\0') { cmd.kind = HL_PING; return cmd; }
  if (line[1] != ',') return cmd;
  int v[4];
  if (line[0] == 'E' && hlParseInts(line + 2, v, 4)) {
    for (int i = 0; i < 4; i++) { if (v[i] < 0) v[i] = 0; if (v[i] > 255) v[i] = 255; }   // Python の _byte と同じ（範囲外は丸める）
    cmd.kind = HL_EYE; cmd.a = v[0]; cmd.b = v[1]; cmd.c = v[2]; cmd.d = v[3];
  } else if (line[0] == 'M' && hlParseInts(line + 2, v, 1)) {
    if (v[0] >= 0 && v[0] <= 3) { cmd.kind = HL_MODE; cmd.a = v[0]; }                      // 0=消灯 1=点灯 2=呼吸 3=まばたき。範囲外は捨てる
  } else if (line[0] == 'L' && hlParseInts(line + 2, v, 2)) {
    if (v[0] >= 0 && v[0] <= 255 && v[1] >= 0 && v[1] <= 255) { cmd.kind = HL_LED; cmd.a = v[0]; cmd.b = v[1]; }   // 範囲外は丸めずに捨てる（光を出す側なので）
  }
  return cmd;
}

// S 行: "S,<uptime_ms>,<tof_mm>,<touch_head>,<touch_back>,<seq>\n"。tof_mm = 0 は測定不能（ranging 未実装のあいだは常に 0）。
static inline int hlFormatSensor(char* buf, size_t n, uint32_t uptime_ms, int tof_mm, bool touch_head, bool touch_back, uint32_t seq) {
  return snprintf(buf, n, "S,%lu,%d,%d,%d,%lu\n", (unsigned long)uptime_ms, tof_mm, touch_head ? 1 : 0, touch_back ? 1 : 0, (unsigned long)(seq % 65536UL));
}

static inline int hlFormatPing(char* buf, size_t n, const char* fw) { return snprintf(buf, n, "P,ok,%s\n", fw); }

// LED のデューティ: 0..255 の指令を上限 max で切る（**上げる方向には変えない**）。
static inline uint8_t hlLedDuty(int cmd, int max_duty) {
  if (cmd < 0) cmd = 0;
  if (cmd > max_duty) cmd = max_duty;
  if (cmd > 255) cmd = 255;
  return (uint8_t)cmd;
}
