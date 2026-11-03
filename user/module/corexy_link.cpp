/**
 * @file corexy_link.cpp
 * @brief 三源入口实现：行解析、帧解析、源仲裁、上报
 */

#include "corexy_link.hpp"

#include <stdlib.h>
#include <string.h>


namespace
{
/* ==================== 小工具 ==================== */

///< 二进制帧头与事件码
constexpr uint8_t HEAD0    = 0xA5U;
constexpr uint8_t HEAD1    = 0x5AU;
constexpr uint8_t EVT_MARK = 0x7FU;

///< 取下一个空白分隔 token，无则返回 false
bool tok(const char *&p, char *out, size_t cap)
{
  while ((*p == ' ') || (*p == '\t'))
  {
    p++;
  }

  if (*p == '\0')
  {
    return false;
  }

  size_t n = 0U;
  while ((*p != ' ') && (*p != '\t') && (*p != '\0'))
  {
    if ((n + 1U) < cap)
    {
      out[n++] = *p;
    }
    p++;
  }

  out[n] = '\0';
  return true;
}

///< 整串十进制，必须完整合法
bool to_int(const char *s, long &v)
{
  char      *end = nullptr;
  const long t   = strtol(s, &end, 10);

  if ((end == s) || (end == nullptr) || (*end != '\0'))
  {
    return false;
  }

  v = t;
  return true;
}

///< 毫米（可带小数）→ 0.01 mm
bool to_centi(const char *s, int32_t &v)
{
  char        *end = nullptr;
  const double d   = strtod(s, &end);

  if ((end == s) || (end == nullptr) || (*end != '\0'))
  {
    return false;
  }

  v = static_cast<int32_t>((d * 100.0) + ((d >= 0.0) ? 0.5 : -0.5));
  return true;
}

///< 帧里的 centi → mm
float to_mm(int32_t centi)
{
  return static_cast<float>(centi) / 100.0f;
}

///< mm → 帧里的 centi（四舍五入）
int16_t to_centi16(float v)
{
  return static_cast<int16_t>((v >= 0.0f) ? ((v * 100.0f) + 0.5f) : ((v * 100.0f) - 0.5f));
}

uint16_t rd_u16(const uint8_t *p)
{
  return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

int32_t rd_i32(const uint8_t *p)
{
  return static_cast<int32_t>((static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
                              (static_cast<uint32_t>(p[2]) << 8) | static_cast<uint32_t>(p[3]));
}

///< 组一帧 `A5 5A | CMD | LEN | PAYLOAD | SUM`
size_t build_frame(uint8_t cmd, const uint8_t *payload, size_t len, uint8_t *out, size_t cap)
{
  if ((len + 5U) > cap)
  {
    return 0U;
  }

  out[0] = HEAD0;
  out[1] = HEAD1;
  out[2] = cmd;
  out[3] = static_cast<uint8_t>(len);

  uint8_t sum = static_cast<uint8_t>(cmd ^ static_cast<uint8_t>(len));

  for (size_t i = 0U; i < len; i++)
  {
    out[4U + i] = payload[i];
    sum         = static_cast<uint8_t>(sum ^ payload[i]);
  }

  out[4U + len] = sum;
  return len + 5U;
}
} // namespace


/* ==================== */
/* ==================== 构造与仲裁 ==================== */

CorexyLink::CorexyLink(const Config &cfg)
  : _cfg(cfg)
{
}

Status CorexyLink::init()
{
  _queue = xQueueCreate(cxy_link::QUEUE_DEPTH, sizeof(CxyCmd));
  if (_queue == nullptr)
  {
    return Status::IO_ERROR;
  }

  // 上电默认值：端口没绑的源一律视为关闭
  _enabled[slot_of(CxySource::Console)] = cxy_link::CONSOLE_ENABLE && (_cfg.console != nullptr);
  _enabled[slot_of(CxySource::Binary)]  = cxy_link::BINARY_ENABLE && (_cfg.binary != nullptr);
  _enabled[slot_of(CxySource::Rc)]      = cxy_link::RC_ENABLE;

  _active   = enabled(cxy_link::DEFAULT_SOURCE) ? cxy_link::DEFAULT_SOURCE : CxySource::None;
  _last_cmd = xTaskGetTickCount();
  return Status::OK;
}

Status CorexyLink::set_enabled(CxySource s, bool on)
{
  if ((s == CxySource::None) || (slot_of(s) > slot_of(CxySource::Rc)))
  {
    return Status::BAD_ARG;
  }

  // 启用需要对应端口已绑定（遥控无端口）
  if (on && (s == CxySource::Console) && (_cfg.console == nullptr))
  {
    return Status::BAD_ARG;
  }

  if (on && (s == CxySource::Binary) && (_cfg.binary == nullptr))
  {
    return Status::BAD_ARG;
  }

  _enabled[slot_of(s)] = on;

  // 关掉当前活动源：退回 None，旧源的残留命令不能继续执行
  if (!on && (_active == s))
  {
    _active = CxySource::None;
    _last_cmd = xTaskGetTickCount();
    reset_parsers();

    if (_queue != nullptr)
    {
      xQueueReset(_queue);
    }
  }

  return Status::OK;
}

bool CorexyLink::enabled(CxySource s) const
{
  return (s != CxySource::None) && _enabled[slot_of(s)];
}

Status CorexyLink::select(CxySource s)
{
  if ((s != CxySource::None) && !enabled(s))
  {
    return Status::BAD_ARG; // 没启用的源不能选
  }

  _active   = s;
  _last_cmd = xTaskGetTickCount();
  reset_parsers();

  // 旧源的残留命令不能在切换后被执行
  if (_queue != nullptr)
  {
    xQueueReset(_queue);
  }

  return Status::OK;
}

void CorexyLink::reset_parsers()
{
  _line_len = 0U;
  _rx_len   = 0U;
}


/* ==================== */
/* ==================== 命令通道 ==================== */

Status CorexyLink::submit(const CxyCmd &cmd)
{
  if ((_queue == nullptr) || (cmd.from != _active) || !enabled(cmd.from))
  {
    return Status::BAD_ARG;
  }

  if (xQueueSend(_queue, &cmd, 0) != pdPASS)
  {
    return Status::FULL;
  }

  _last_cmd = xTaskGetTickCount();
  return Status::OK;
}

bool CorexyLink::take(CxyCmd &out)
{
  return (_queue != nullptr) && (xQueueReceive(_queue, &out, 0) == pdPASS);
}

bool CorexyLink::timed_out() const
{
  if ((_cfg.cmd_timeout_ms == 0U) || (_active == CxySource::None))
  {
    return false;
  }

  return (xTaskGetTickCount() - _last_cmd) > pdMS_TO_TICKS(_cfg.cmd_timeout_ms);
}

void CorexyLink::reset_timeout()
{
  _last_cmd = xTaskGetTickCount();
}


/* ==================== */
/* ==================== 终端口 ==================== */

void CorexyLink::feed_console(const uint8_t *data, size_t size)
{
  if (!enabled(CxySource::Console) || (_active != CxySource::Console))
  {
    return; // 未启用 / 非活动源：字节照读、只丢不解析
  }

  for (size_t i = 0U; i < size; i++)
  {
    const char c = static_cast<char>(data[i]);

    if ((c == '\r') || (c == '\n') || (c == ';'))
    {
      _line[_line_len] = '\0';
      parse_line(_line);
      _line_len = 0U;
      continue;
    }

    if (_line_len < (cxy_link::LINE_MAX - 1U))
    {
      _line[_line_len++] = c;
    }
    else
    {
      _line_len = 0U; // 超长行直接丢弃
    }
  }
}

void CorexyLink::parse_line(const char *text)
{
  const char *p = text;
  char        c[8];

  if (!tok(p, c, sizeof(c)))
  {
    return;
  }

  if ((strcmp(c, "?") == 0) || (strcmp(c, "help") == 0))
  {
    print_help();
    return;
  }

  CxyCmd cmd {};

  if (strcmp(c, "pos") == 0)
  {
    cmd.type = CxyCmd::Type::Status;
  }
  else if (strcmp(c, "dump") == 0)
  {
    cmd.type = CxyCmd::Type::Dump;
  }
  else if (strcmp(c, "lim") == 0)
  {
    cmd.type = CxyCmd::Type::Limits;
  }
  else if ((strcmp(c, "home") == 0) || (strcmp(c, "zero") == 0))
  {
    cmd.type = CxyCmd::Type::Zero;
  }
  else if (strcmp(c, "stop") == 0)
  {
    cmd.type = CxyCmd::Type::Stop;
  }
  else if ((strcmp(c, "m") == 0) || (strcmp(c, "j") == 0))
  {
    char    sx[16];
    char    sy[16];
    int32_t cx = 0;
    int32_t cy = 0;

    if (!tok(p, sx, sizeof(sx)) || !tok(p, sy, sizeof(sy)) || !to_centi(sx, cx) || !to_centi(sy, cy))
    {
      print_console((strcmp(c, "m") == 0) ? "[CXY] usage: m <x> <y>\r\n" : "[CXY] usage: j <dx> <dy>\r\n");
      return;
    }

    cmd.type = (strcmp(c, "m") == 0) ? CxyCmd::Type::MoveAbs : CxyCmd::Type::MoveRel;
    cmd.a    = to_mm(cx);
    cmd.b    = to_mm(cy);
  }
  else if (strcmp(c, "g") == 0)
  {
    // 0 = +X，逆时针每档 45°
    static const float DX[8] = {1.0f, 0.7071f, 0.0f, -0.7071f, -1.0f, -0.7071f, 0.0f, 0.7071f};
    static const float DY[8] = {0.0f, 0.7071f, 1.0f, 0.7071f, 0.0f, -0.7071f, -1.0f, -0.7071f};

    char    sd[8];
    char    sm[16];
    long    dir = 0;
    int32_t cmm = 1000; // 默认 10 mm

    if (!tok(p, sd, sizeof(sd)) || !to_int(sd, dir) || (dir < 0) || (dir > 7))
    {
      print_console("[CXY] usage: g <0..7> [mm]\r\n");
      return;
    }

    if (tok(p, sm, sizeof(sm)) && (!to_centi(sm, cmm)))
    {
      print_console("[CXY] usage: g <0..7> [mm]\r\n");
      return;
    }

    const float d = to_mm(cmm);
    cmd.type      = CxyCmd::Type::MoveRel;
    cmd.a         = d * DX[dir];
    cmd.b         = d * DY[dir];
  }
  else if (strcmp(c, "spd") == 0)
  {
    char sr[16];
    char sa[16];
    long rpm = 0;
    long acc = 0;

    if (!tok(p, sr, sizeof(sr)) || !tok(p, sa, sizeof(sa)) || !to_int(sr, rpm) || !to_int(sa, acc) || (rpm < 1) || (rpm > 5000) ||
        (acc < 0) || (acc > 255))
    {
      print_console("[CXY] usage: spd <rpm 1..5000> <acc 0..255>\r\n");
      return;
    }

    cmd.type = CxyCmd::Type::SetSpeed;
    cmd.p0   = static_cast<int32_t>(rpm);
    cmd.p1   = static_cast<int32_t>(acc);
  }
  else if (strcmp(c, "en") == 0)
  {
    char son[8];
    long on = 0;

    if (!tok(p, son, sizeof(son)) || !to_int(son, on) || ((on != 0) && (on != 1)))
    {
      print_console("[CXY] usage: en <0|1>\r\n");
      return;
    }

    cmd.type = CxyCmd::Type::Enable;
    cmd.p0   = static_cast<int32_t>(on);
  }
  else if (strcmp(c, "qp") == 0)
  {
    char sa[8];
    char sm[8];
    char sr[16];
    char sc[16];
    long addr = 0;
    long rpm  = 0;
    long acc  = 0;

    if (!tok(p, sa, sizeof(sa)) || !tok(p, sm, sizeof(sm)) || !tok(p, sr, sizeof(sr)) || !tok(p, sc, sizeof(sc)) ||
        !to_int(sa, addr) || !to_int(sr, rpm) || !to_int(sc, acc) || (addr < 1) || (addr > 2))
    {
      print_console("[CXY] usage: qp <1|2> <rel|abs> <rpm> <acc>\r\n");
      return;
    }

    uint8_t m = 0U;
    if (strcmp(sm, "rel") == 0)
    {
      m = 0U;
    }
    else if (strcmp(sm, "abs") == 0)
    {
      m = 1U;
    }
    else
    {
      print_console("[CXY] usage: qp <1|2> <rel|abs> <rpm> <acc>\r\n");
      return;
    }

    cmd.type = CxyCmd::Type::QuickParam;
    cmd.addr = static_cast<uint8_t>(addr);
    cmd.p0   = static_cast<int32_t>(rpm);
    cmd.p1   = static_cast<int32_t>(acc);
    cmd.mode = m;
  }
  else if (strcmp(c, "q") == 0)
  {
    char sa[8];
    char sp[16];
    long addr   = 0;
    long pulses = 0;

    if (!tok(p, sa, sizeof(sa)) || !tok(p, sp, sizeof(sp)) || !to_int(sa, addr) || !to_int(sp, pulses) || (addr < 1) || (addr > 2))
    {
      print_console("[CXY] usage: q <1|2> <pulses>\r\n");
      return;
    }

    cmd.type = CxyCmd::Type::QuickPos;
    cmd.addr = static_cast<uint8_t>(addr);
    cmd.p0   = static_cast<int32_t>(pulses);
  }
  else if (strcmp(c, "src") == 0)
  {
    char ss[8];
    long s = 0;

    if (!tok(p, ss, sizeof(ss)) || !to_int(ss, s) || (s < 0) || (s > 2))
    {
      print_console("[CXY] usage: src <0 终端 | 1 二进制 | 2 遥控>\r\n");
      return;
    }

    const CxySource target = (s == 0) ? CxySource::Console : ((s == 1) ? CxySource::Binary : CxySource::Rc);
    (void)select(target);
    print_console("[CXY] source switched\r\n");
    return;
  }
  else
  {
    print_console("[CXY] unknown, try ?\r\n");
    return;
  }

  cmd.from = CxySource::Console;

  if (submit(cmd) != Status::OK)
  {
    print_console("[CXY] busy: queue full\r\n");
  }
}

void CorexyLink::print_help()
{
  print_console("[CXY] m <x> <y>       绝对移动 (mm)\r\n");
  print_console("[CXY] j <dx> <dy>     相对移动 (mm)\r\n");
  print_console("[CXY] g <0..7> [mm]   八方向单步\r\n");
  print_console("[CXY] pos | lim | dump | home | stop\r\n");
  print_console("[CXY] en <0|1>        失能 / 使能\r\n");
  print_console("[CXY] spd <rpm> <acc> 速度 (1..5000, 0..255)\r\n");
  print_console("[CXY] qp <1|2> <rel|abs> <rpm> <acc>   快速位置参数\r\n");
  print_console("[CXY] q <1|2> <pulses>                 快速位置\r\n");
  print_console("[CXY] src <0|1|2>     切源：终端 / 二进制 / 遥控\r\n");
}


/* ==================== */
/* ==================== 二进制口 ==================== */

void CorexyLink::feed_binary(const uint8_t *data, size_t size)
{
  if (!enabled(CxySource::Binary) || (_active != CxySource::Binary))
  {
    return; // 未启用 / 非活动源：字节照读、只丢不解析
  }

  for (size_t i = 0U; i < size; i++)
  {
    const uint8_t b = data[i];

    if (_rx_len == 0U)
    {
      if (b == HEAD0)
      {
        _rx[_rx_len++] = b;
      }
      continue;
    }

    if (_rx_len == 1U)
    {
      if (b == HEAD1)
      {
        _rx[_rx_len++] = b;
      }
      else if (b != HEAD0)
      {
        _rx_len = 0U; // 帧头错位，重新同步
      }
      continue;
    }

    _rx[_rx_len++] = b;

    if (_rx_len == 4U)
    {
      if (_rx[3] > cxy_link::PAYLOAD_MAX)
      {
        _rx_len = 0U; // 长度非法
      }
      continue;
    }

    if (_rx_len != (static_cast<size_t>(_rx[3]) + 5U))
    {
      continue;
    }

    const uint8_t cmd = _rx[2];
    const size_t  len = _rx[3];

    uint8_t sum = static_cast<uint8_t>(cmd ^ static_cast<uint8_t>(len));
    for (size_t k = 0U; k < len; k++)
    {
      sum = static_cast<uint8_t>(sum ^ _rx[4U + k]);
    }

    if (sum == _rx[_rx_len - 1U])
    {
      parse_binary_frame(cmd, _rx + 4U, len);
    }

    _rx_len = 0U;
  }
}

void CorexyLink::parse_binary_frame(uint8_t cmd, const uint8_t *payload, size_t len)
{
  CxyCmd c {};
  c.type = static_cast<CxyCmd::Type>(cmd);
  c.from = CxySource::Binary;

  switch (c.type)
  {
    case CxyCmd::Type::Enable:
      if (len < 1U)
      {
        return;
      }
      c.p0 = (payload[0] != 0U) ? 1 : 0;
      break;

    case CxyCmd::Type::MoveAbs:
    case CxyCmd::Type::MoveRel:
      if (len < 4U)
      {
        return;
      }
      c.a = to_mm(static_cast<int16_t>(rd_u16(payload)));
      c.b = to_mm(static_cast<int16_t>(rd_u16(payload + 2U)));
      break;

    case CxyCmd::Type::SetSpeed:
      if (len < 3U)
      {
        return;
      }
      c.p0 = static_cast<int32_t>(rd_u16(payload));
      c.p1 = static_cast<int32_t>(payload[2]);
      break;

    case CxyCmd::Type::QuickParam:
      if (len < 5U)
      {
        return;
      }
      c.addr = payload[0];
      c.p0   = static_cast<int32_t>(rd_u16(payload + 1U));
      c.p1   = static_cast<int32_t>(payload[3]);
      c.mode = payload[4];
      break;

    case CxyCmd::Type::QuickPos:
      if (len < 5U)
      {
        return;
      }
      c.addr = payload[0];
      c.p0   = rd_i32(payload + 1U);
      break;

    case CxyCmd::Type::Zero:
    case CxyCmd::Type::Stop:
    case CxyCmd::Type::Limits:
      break;

    default:
      return; // 未定义命令直接丢弃
  }

  (void)submit(c);
}


/* ==================== */
/* ==================== 上报 ==================== */

void CorexyLink::reply(const CxyCmd &cmd, Status st, float x, float y)
{
  if ((cmd.from == CxySource::Binary) && (_cfg.binary != nullptr))
  {
    const int16_t xi     = to_centi16(x);
    const int16_t yi     = to_centi16(y);
    const uint8_t p[5]   = {static_cast<uint8_t>(st), static_cast<uint8_t>(static_cast<uint16_t>(xi) >> 8),
                            static_cast<uint8_t>(xi), static_cast<uint8_t>(static_cast<uint16_t>(yi) >> 8), static_cast<uint8_t>(yi)};
    uint8_t       buf[16] = {};
    const size_t  n      = build_frame(static_cast<uint8_t>(code_of(cmd.type) | 0x80U), p, sizeof(p), buf, sizeof(buf));

    if (n > 0U)
    {
      (void)_cfg.binary->send(buf, n, pdMS_TO_TICKS(10U), nullptr);
    }
  }
  else if ((cmd.from == CxySource::Console) && (_cfg.console != nullptr))
  {
    _cfg.console->printf("[CXY] cmd 0x%02X -> st=%u\r\n", static_cast<unsigned>(code_of(cmd.type)), static_cast<unsigned>(st));
  }
}

void CorexyLink::reply_limits(Status st, const float theory[4], const float safe[4])
{
  if (_active == CxySource::Binary)
  {
    uint8_t p[17] = {};
    p[0]          = static_cast<uint8_t>(st);

    for (size_t i = 0U; i < 4U; i++)
    {
      const int16_t t = to_centi16(theory[i]);
      const int16_t s = to_centi16(safe[i]);

      p[1U + (2U * i)]   = static_cast<uint8_t>(static_cast<uint16_t>(t) >> 8);
      p[2U + (2U * i)]   = static_cast<uint8_t>(t);
      p[9U + (2U * i)]   = static_cast<uint8_t>(static_cast<uint16_t>(s) >> 8);
      p[10U + (2U * i)]  = static_cast<uint8_t>(s);
    }

    uint8_t      buf[24] = {};
    const size_t n       = build_frame(static_cast<uint8_t>(code_of(CxyCmd::Type::Limits) | 0x80U), p, sizeof(p), buf, sizeof(buf));

    if ((n > 0U) && (_cfg.binary != nullptr))
    {
      (void)_cfg.binary->send(buf, n, pdMS_TO_TICKS(10U), nullptr);
    }
    return;
  }

  if (_cfg.console != nullptr)
  {
    _cfg.console->printf("[CXY] theory=(%.2f,%.2f)-(%.2f,%.2f) safe=(%.2f,%.2f)-(%.2f,%.2f)\r\n", theory[0], theory[1], theory[2],
                         theory[3], safe[0], safe[1], safe[2], safe[3]);
  }
}

void CorexyLink::event(uint8_t code, Status st, float x, float y)
{
  if ((_active == CxySource::Binary) && (_cfg.binary != nullptr))
  {
    const int16_t xi      = to_centi16(x);
    const int16_t yi      = to_centi16(y);
    const uint8_t p[6]    = {code, static_cast<uint8_t>(st), static_cast<uint8_t>(static_cast<uint16_t>(xi) >> 8),
                             static_cast<uint8_t>(xi), static_cast<uint8_t>(static_cast<uint16_t>(yi) >> 8), static_cast<uint8_t>(yi)};
    uint8_t       buf[16] = {};
    const size_t  n       = build_frame(EVT_MARK, p, sizeof(p), buf, sizeof(buf));

    if (n > 0U)
    {
      (void)_cfg.binary->send(buf, n, pdMS_TO_TICKS(10U), nullptr);
    }
    return;
  }

  if (_cfg.console != nullptr)
  {
    _cfg.console->printf("[CXY] %s st=%u pos=(%.2f, %.2f)\r\n",
                         (code == 0x01U) ? "boot" : ((code == 0x02U) ? "done" : "fault"), static_cast<unsigned>(st), x, y);
  }
}

void CorexyLink::print_console(const char *text)
{
  if (_cfg.console != nullptr)
  {
    _cfg.console->printf("%s", text);
  }
}

/* ==================== */
