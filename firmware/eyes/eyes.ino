/*
  Göz anahtarlık – ESP32-S3 SuperMini + MPU6050 + 0.96" SSD1306 (I2C)

  Bağlantı:
    3V3 -> OLED VCC, MPU VCC      GND -> OLED GND, MPU GND, MPU AD0
    GPIO8 -> SDA (ikisi)          GPIO9 -> SCL (ikisi)
    GPIO7 -> MPU INT (şimdilik kullanılmıyor, uyku/uyandırma için ayrıldı)

  Ek kütüphane gerekmez (ekran ve MPU6050 sürücüsü bu dosyada).
  Arduino IDE: Kart = "ESP32S3 Dev Module", USB CDC On Boot = Enabled.

  Seri monitör (115200): 0-9 tuşu ifadeyi 6 sn zorlar, r = göz devir,
  c = şu anki duruşu merkez yap. Eksen yönleri ters ise aşağıdaki SGN_* değerlerini değiştir.
*/
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ================= AYARLAR =================
#define PIN_SDA      8
#define PIN_SCL      9
#define OLED_ADDR    0x3C
#define MPU_ADDR     0x68
#define I2C_HZ       400000   // ekran destekliyorsa 800000 dene (daha akıcı)
#define OLED_CONTRAST 0x8F    // 0x00-0xFF, düşük = az akım

// MPU6050 eksenlerinin ekrana göre yönü. GY-521 ekranın arkasında, yazıları
// ekranla aynı yöne bakacak şekilde yatıksa varsayılanlar uygundur.
#define AX_RIGHT 0   // sensörün hangi ekseni ekranın SAĞI (0=X,1=Y,2=Z)
#define AX_UP    1   // hangi ekseni ekranın ÜSTÜ
#define AX_OUT   2   // hangi ekseni ekrandan DIŞARI (izleyiciye doğru)
#define SGN_RIGHT 1  // gözler eğime ters gidiyorsa -1 yap
#define SGN_UP    1
#define SGN_OUT   1

// ================= EKRAN SÜRÜCÜSÜ =================
static const int W = 128, H = 64;
static uint8_t fb[W * H / 8];
static inline void setPx(int x, int y, bool on) {
  if ((unsigned)x >= (unsigned)W || (unsigned)y >= (unsigned)H) return;
  uint8_t &b = fb[x + (y >> 3) * W];
  if (on) b |= 1 << (y & 7); else b &= ~(1 << (y & 7));
}
static void oledCmd(uint8_t c) { Wire.beginTransmission(OLED_ADDR); Wire.write(0x00); Wire.write(c); Wire.endTransmission(); }
static void oledInit() {
  const uint8_t init[] = {0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x8D, 0x14, 0x20, 0x00,
                          0xA1, 0xC8, 0xDA, 0x12, 0x81, OLED_CONTRAST, 0xD9, 0xF1, 0xDB, 0x40,
                          0xA4, 0xA6, 0x2E, 0xAF};
  for (uint8_t c : init) oledCmd(c);
}
static void oledFlush() {
  oledCmd(0x21); oledCmd(0); oledCmd(127);
  oledCmd(0x22); oledCmd(0); oledCmd(7);
  for (int i = 0; i < (int)sizeof(fb); i += 32) {
    Wire.beginTransmission(OLED_ADDR); Wire.write(0x40);
    Wire.write(fb + i, 32);
    Wire.endTransmission();
  }
}

// ================= YARDIMCILAR =================
static float frand() { return (float)esp_random() / 4294967295.0f; }
static float rnd(float a, float b) { return a + (b - a) * frand(); }
static float clampf(float v, float a, float b) { return v < a ? a : v > b ? b : v; }
static const float PI_F = 3.14159265f;

// Basit çizim dönüşümü (canvas translate/scale gibi)
struct Xf { float ox, oy, sx, sy; };
static Xf X = {0, 0, 1, 1};

template <typename F>
static void fillLocal(float x0, float y0, float x1, float y1, bool on, F inside) {
  float ax = X.ox + x0 * X.sx, bx = X.ox + x1 * X.sx, ay = X.oy + y0 * X.sy, by = X.oy + y1 * X.sy;
  if (ax > bx) { float t = ax; ax = bx; bx = t; }
  if (ay > by) { float t = ay; ay = by; by = t; }
  int px0 = max(0, (int)floorf(ax)), px1 = min(W - 1, (int)ceilf(bx));
  int py0 = max(0, (int)floorf(ay)), py1 = min(H - 1, (int)ceilf(by));
  for (int py = py0; py <= py1; py++)
    for (int px = px0; px <= px1; px++) {
      float u = (px + 0.5f - X.ox) / X.sx, v = (py + 0.5f - X.oy) / X.sy;
      if (inside(u, v)) setPx(px, py, on);
    }
}
static float segDist(float u, float v, float ax, float ay, float bx, float by) {
  float dx = bx - ax, dy = by - ay, l = dx * dx + dy * dy;
  float t = l > 0 ? clampf(((u - ax) * dx + (v - ay) * dy) / l, 0, 1) : 0;
  float ex = ax + t * dx - u, ey = ay + t * dy - v;
  return sqrtf(ex * ex + ey * ey);
}
static void dot(float cx, float cy, float rx, float ry, bool on = true) {
  fillLocal(cx - rx, cy - ry, cx + rx, cy + ry, on, [&](float u, float v) {
    float a = (u - cx) / rx, b = (v - cy) / ry; return a * a + b * b <= 1; });
}
static void poly(const float *p, int n, float w) {
  float x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
  for (int i = 0; i < n; i++) { x0 = min(x0, p[2*i]); x1 = max(x1, p[2*i]); y0 = min(y0, p[2*i+1]); y1 = max(y1, p[2*i+1]); }
  float h = w / 2;
  fillLocal(x0 - h, y0 - h, x1 + h, y1 + h, true, [&](float u, float v) {
    for (int i = 0; i + 1 < n; i++) if (segDist(u, v, p[2*i], p[2*i+1], p[2*i+2], p[2*i+3]) <= h) return true;
    return false; });
}
static void seg(float ax, float ay, float bx, float by, float w) { float p[] = {ax, ay, bx, by}; poly(p, 2, w); }
// canvas arc(): açı +x'ten saat yönünde (y aşağı)
static void arcStroke(float cx, float cy, float r, float a0, float a1, float w) {
  float h = w / 2;
  float ex0 = cx + r * cosf(a0), ey0 = cy + r * sinf(a0), ex1 = cx + r * cosf(a1), ey1 = cy + r * sinf(a1);
  fillLocal(cx - r - h, cy - r - h, cx + r + h, cy + r + h, true, [&](float u, float v) {
    float du = u - cx, dv = v - cy, d = sqrtf(du * du + dv * dv);
    float a = atan2f(dv, du); if (a < 0) a += 2 * PI_F;
    if (a >= a0 && a <= a1 && fabsf(d - r) <= h) return true;
    float q0 = (u-ex0)*(u-ex0)+(v-ey0)*(v-ey0), q1 = (u-ex1)*(u-ex1)+(v-ey1)*(v-ey1);
    return q0 <= h * h || q1 <= h * h; });
}
static void halfDisk(float cx, float cy, float r) {  // alt yarım daire (D ağız)
  fillLocal(cx - r, cy, cx + r, cy + r, true, [&](float u, float v) {
    float a = u - cx, b = v - cy; return b >= 0 && a * a + b * b <= r * r; });
}
static void ring(float rx, float ry, float w) {
  float h = w / 2;
  fillLocal(-rx - h, -ry - h, rx + h, ry + h, true, [&](float u, float v) {
    float o = (u*u)/((rx+h)*(rx+h)) + (v*v)/((ry+h)*(ry+h));
    float i = (u*u)/((rx-h)*(rx-h)) + (v*v)/((ry-h)*(ry-h));
    return o <= 1 && i >= 1; });
}
// iki nokta arasındaki çizginin ÜSTÜNÜ siyahla (kızgın/üzgün kesim, göz kapağı)
static void cutAbove(float xa, float ya, float xb, float yb, float top) {
  float x0 = min(xa, xb), x1 = max(xa, xb);
  fillLocal(x0, top, x1, max(ya, yb), false, [&](float u, float v) {
    float t = (u - xa) / (xb - xa); return v <= ya + (yb - ya) * t; });
}
static void heart() {
  fillLocal(-11, -10, 11, 8, true, [&](float u, float v) {
    float x = u / 8.5f, y = -(v + 1.0f) / 7.0f, q = x * x + y * y - 1;
    return q * q * q - x * x * y * y * y <= 0; });
}
static void stampScreen(float sx, float sy, float r) {
  for (int y = (int)floorf(sy - r); y <= (int)ceilf(sy + r); y++)
    for (int x = (int)floorf(sx - r); x <= (int)ceilf(sx + r); x++) {
      float a = x + 0.5f - sx, b = y + 0.5f - sy; if (a * a + b * b <= r * r) setPx(x, y, true); }
}
static void stampLocal(float u, float v, float wLocal) {
  stampScreen(X.ox + u * X.sx, X.oy + v * X.sy, max(0.6f, wLocal / 2 * (fabsf(X.sx) + fabsf(X.sy)) / 2));
}

// ================= İFADELER =================
enum Mood { NORMAL, HAPPY, WINK, SAD, ANGRY, SURPRISE, SLEEP, LOVE, DIZZY, FAINT, MOOD_N };
static const char *MOOD_NAME[] = {"normal", "mutlu", "goz kirp", "uzgun", "kizgin", "saskin", "uyku", "asik", "sersem", "bayildi"};
static const bool BLUSH[] = {0, 1, 1, 0, 0, 0, 0, 1, 0, 0};
static const bool NOBLINK[] = {0, 0, 0, 0, 0, 0, 1, 1, 1, 1};

static void drawEye(Mood m, int s, float t) {
  switch (m) {
    case NORMAL: dot(0, 0, 6, 8); break;
    case HAPPY: arcStroke(0, 5, 9, PI_F * 1.15f, PI_F * 1.85f, 3.5f); break;
    case WINK:
      if (s < 0) dot(0, 0, 6, 8);
      else { float p[] = {7, -7, -6, 0, 7, 7}; poly(p, 3, 3.5f); }
      break;
    case SAD: dot(0, 0, 6, 7); cutAbove(s * 12, 0, -s * 12, -9, -12); break;
    case ANGRY: dot(0, 0, 9, 8); cutAbove(-s * 14, 2, s * 14, -9, -14); break;
    case SURPRISE: dot(0, 0, 8, 11); break;
    case SLEEP: arcStroke(0, -5, 8, PI_F * 0.15f, PI_F * 0.85f, 3); break;
    case LOVE: {
      float k = 1 + powf(fmaxf(0, sinf(t / 160)), 8) * 0.25f;
      Xf b = X; X.sx *= k; X.sy *= k; heart(); X = b; break;
    }
    case DIZZY: {
      float a = t / 120 * s;
      for (float u = 0; u < 15; u += 0.15f) stampLocal(cosf(u + a) * u * 0.65f, sinf(u + a) * u * 0.65f, 2);
      break;
    }
    case FAINT: seg(-6, -6, 6, 6, 3.5f); seg(6, -6, -6, 6, 3.5f); break;
    default: break;
  }
}
static void drawMouth(Mood m, float t) {
  switch (m) {
    case NORMAL: arcStroke(0, -4, 5, PI_F * 0.2f, PI_F * 0.8f, 2.5f); break;
    case HAPPY: arcStroke(0, -5, 7, PI_F * 0.2f, PI_F * 0.8f, 2.5f); break;
    case WINK: case LOVE: halfDisk(0, -2, 6); break;
    case SAD: arcStroke(0, 5, 6, PI_F * 1.2f, PI_F * 1.8f, 2.5f); break;
    case ANGRY: seg(-5, 1, 5, 1, 2.5f); break;
    case SURPRISE: ring(3, 3.5f + sinf(t / 150) * 0.6f, 2.5f); break;
    case SLEEP: arcStroke(0, 3, 3, PI_F * 1.15f, PI_F * 1.85f, 2); break;
    case DIZZY: for (float i = -7; i <= 7; i += 0.25f) stampLocal(i, sinf(i * 0.9f + t / 90) * 1.5f, 2); break;
    case FAINT: { float p[26]; for (int i = 0; i < 13; i++) { p[2*i] = i - 6; p[2*i+1] = (i & 1) ? 1 : -1; } poly(p, 13, 2); break; }
    default: break;
  }
}
static void blush(float cx, float cy) {
  Xf b = X; X = {0, 0, 1, 1};
  for (int i = 0; i < 3; i++) seg(cx + i * 4 - 4, cy + 2, cx + i * 4 - 2, cy - 1, 1.6f);
  X = b;
}
static void drawZ(int x, int y, int s) {  // küçük z harfi
  for (int i = 0; i < s; i++) { setPx(x + i, y, 1); setPx(x + i, y + s - 1, 1); setPx(x + s - 1 - i, y + i, 1); }
}

// ================= DURUM =================
static Mood mood = NORMAL, pending = NORMAL; static bool hasPending = false;
static uint32_t moodUntil = 0; static float pop = 0;
static Mood forced = NORMAL; static bool hasForced = false; static uint32_t forcedUntil = 0;
static Mood target() { return hasPending ? pending : mood; }

static float blinkT = -1e9; static bool dbl = false; static float nextBlink = 2500;
static void startBlink(float now, bool d) { blinkT = now; dbl = d; nextBlink = now + rnd(2500, 6500); }
static void setMood(Mood m) { if (m == target()) return; pending = m; hasPending = true; startBlink(millis(), false); }
static void trigger(Mood m, float ms) { if (target() != m) setMood(m); moodUntil = millis() + (uint32_t)ms; }
static float blinkAmt(float now) {
  float t = now - blinkT, b = 0;
  if (t < 60) b = t / 60; else if (t < 90) b = 1; else if (t < 200) b = 1 - (t - 90) / 110;
  else if (dbl && t < 280) b = 0;
  else if (dbl && t < 480) { float u = t - 280; b = u < 55 ? u / 55 : u < 80 ? 1 : fmaxf(0, 1 - (u - 80) / 110); }
  return b * b * (3 - 2 * b);
}

// bakış (yay fiziği)
static float gzx = 0, gzy = 0, gvx = 0, gvy = 0;
static float tiltX = 0, tiltY = 0, moving = 0;
static float sacX = 0, sacY = 0, nextSac = 1500, driftX = 0, driftY = 0;
struct Scan { bool on; float x0, x1, y, t0, dur; } scan = {false};
struct Roll { bool on; float t0, dur; int dir; } roll = {false};
static void startRoll(float t) { roll = {true, t, rnd(1100, 1500), frand() < 0.5f ? -1 : 1}; }

// duygu motoru
struct Brain { float energy = 0.8f, joy = 0.5f, anger = 0, dizzy = 0, bored = 0, hold = 0, next = 0, spont = 20000, sulkUntil = 0; } B;
static bool faceDown = false;
static void brainFeed(float lin, float rot, float dts) {
  float g = fminf(1, fmaxf(0, (lin - 0.4f) / 3)) + fminf(1, rot / 120) * 0.5f;
  B.energy = fminf(1, B.energy + g * 0.12f * dts);
  if (g > 0.3f) B.bored = fmaxf(0, B.bored - g * 0.5f * dts);
  if (lin < 5) B.joy = fminf(1, B.joy + g * 0.05f * dts);
}
static void brainStep(float now, float dts) {
  B.energy = fmaxf(0, B.energy - 0.006f * dts);
  B.anger = fmaxf(0, B.anger - 0.06f * dts);
  B.dizzy = fmaxf(0, B.dizzy - 0.1f * dts);
  B.joy += (0.45f - B.joy) * 0.01f * dts;
  if (moving < 0.15f && mood != SLEEP) B.bored += dts / 35;
  if (B.bored > 1 && !roll.on && (mood == NORMAL || mood == SAD) && !hasPending && B.energy > 0.3f) { startRoll(now); B.bored = 0.35f; }
  if (hasForced && now < forcedUntil) { if (target() != forced) setMood(forced); return; }
  hasForced = false;
  if (moodUntil && now < moodUntil) return;
  moodUntil = 0;
  if (now < B.next) return;
  B.next = now + 400;
  Mood want;
  if (faceDown) want = FAINT;
  else if (B.dizzy > 0.35f) want = DIZZY;
  else if (B.anger > 0.45f) { want = ANGRY; B.sulkUntil = now + rnd(4000, 7000); }
  else if (now < B.sulkUntil) { want = SAD; if (mood == ANGRY && !roll.on && frand() < 0.6f) startRoll(now + 600); }
  else if (B.energy < 0.12f || (mood == SLEEP && B.energy < 0.3f)) want = SLEEP;
  else if (B.joy > 0.9f) want = LOVE;
  else if (B.joy > 0.7f) want = HAPPY;
  else want = NORMAL;
  if (want == NORMAL && now > B.spont) {
    float r = frand();
    if (r < 0.45f) trigger(HAPPY, rnd(2500, 4500));
    else if (r < 0.65f && B.joy > 0.4f) trigger(WINK, 1400);
    else if (r < 0.8f && B.energy < 0.45f) trigger(SLEEP, rnd(1500, 2500));
    B.spont = now + rnd(14000, 32000);
    return;
  }
  if (want != target() && (now > B.hold || want == FAINT || want == ANGRY || want == DIZZY)) {
    if (mood == SLEEP && want != SLEEP && want != FAINT) { trigger(SURPRISE, 1800); B.hold = now + 1800; return; }
    setMood(want); B.hold = now + 3000;
  }
}

// ================= MPU6050 =================
static bool mpuOk = false;
static float grav[3] = {0, 0, 9.81f}, base[3] = {0, 0, 9.81f};
static bool haveBase = false; static uint32_t bootMs;
static uint32_t shakePeaks[8]; static int nPeaks = 0; static int lastPeakSign = 0;
static float spinMs = 0, downSince = 0, tapAt = 0, linMag = 0, rotMag = 0;

static void mpuWrite(uint8_t r, uint8_t v) { Wire.beginTransmission(MPU_ADDR); Wire.write(r); Wire.write(v); Wire.endTransmission(); }
static void mpuInit() {
  mpuWrite(0x6B, 0x01);  // uyandır, PLL saat
  mpuWrite(0x1A, 0x03);  // DLPF ~44 Hz
  mpuWrite(0x1B, 0x08);  // gyro ±500 °/s
  mpuWrite(0x1C, 0x08);  // ivme ±4 g
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x75); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1); mpuOk = Wire.available() && Wire.read() != 0xFF;
}
static bool mpuRead(float a[3], float g[3]) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, 14) != 14) return false;
  int16_t v[7];
  for (int i = 0; i < 7; i++) { uint8_t h = Wire.read(), l = Wire.read(); v[i] = (int16_t)((h << 8) | l); }
  float ra[3] = {v[0] * 9.81f / 8192, v[1] * 9.81f / 8192, v[2] * 9.81f / 8192};
  float rg[3] = {v[4] / 65.5f, v[5] / 65.5f, v[6] / 65.5f};
  a[0] = SGN_RIGHT * ra[AX_RIGHT]; a[1] = SGN_UP * ra[AX_UP]; a[2] = SGN_OUT * ra[AX_OUT];
  g[0] = rg[AX_RIGHT]; g[1] = rg[AX_UP]; g[2] = rg[AX_OUT];
  return true;
}
static int countPeaks(uint32_t now) {
  int k = 0; for (int i = 0; i < nPeaks; i++) if (now - shakePeaks[i] < 1000) shakePeaks[k++] = shakePeaks[i];
  return nPeaks = k;
}
static void sensorStep(uint32_t now, float dtm) {
  float a[3], g[3];
  if (!mpuOk || !mpuRead(a, g)) return;
  for (int i = 0; i < 3; i++) grav[i] += (a[i] - grav[i]) * 0.1f;
  if (!haveBase && now - bootMs > 600) { for (int i = 0; i < 3; i++) base[i] = grav[i]; haveBase = true; }
  float la[3] = {a[0] - grav[0], a[1] - grav[1], a[2] - grav[2]};
  linMag = sqrtf(la[0]*la[0] + la[1]*la[1] + la[2]*la[2]);
  rotMag = sqrtf(g[0]*g[0] + g[1]*g[1] + g[2]*g[2]);
  // eğim -> bakış (ölü bölge + yumuşatma)
  float nx = clampf(-(grav[0] - base[0]) / (9.81f * 0.47f), -1, 1);
  float ny = clampf((grav[1] - base[1]) / (9.81f * 0.47f), -1, 1);
  auto dz = [](float v) { return fabsf(v) < 0.08f ? 0.0f : (v - (v > 0 ? 0.08f : -0.08f)) / 0.92f; };
  nx = dz(nx); ny = dz(ny);
  moving = fminf(1, moving + (fabsf(nx - tiltX) + fabsf(ny - tiltY)) * 3);
  tiltX += (nx - tiltX) * 0.25f; tiltY += (ny - tiltY) * 0.25f;
  brainFeed(linMag, rotMag, dtm / 1000);
  // ters çevirme: yerçekimi, açılıştaki duruşun tersine dönmüş
  float gb = grav[0]*base[0] + grav[1]*base[1] + grav[2]*base[2];
  float gm = sqrtf(grav[0]*grav[0]+grav[1]*grav[1]+grav[2]*grav[2]) * sqrtf(base[0]*base[0]+base[1]*base[1]+base[2]*base[2]);
  if (haveBase && gm > 0 && gb / gm < -0.5f) { if (!downSince) downSince = now; faceDown = now - downSince > 600; }
  else { downSince = 0; faceDown = false; }
  // döndürme
  spinMs = rotMag > 220 ? spinMs + dtm : fmaxf(0, spinMs - dtm * 2);
  if (spinMs > 350) { B.dizzy = fminf(1, B.dizzy + 0.7f); B.energy = 1; spinMs = 0; }
  // sallama
  float ax = fabsf(la[0]) > fabsf(la[1]) ? la[0] : la[1];
  int np = countPeaks(now);
  int sg = ax > 0 ? 1 : -1;
  if (fabsf(ax) > 9 && sg != lastPeakSign && np < 8) { lastPeakSign = sg; shakePeaks[nPeaks++] = now; np++; }
  if (np >= 4) { B.anger = fminf(1, B.anger + 0.5f); B.joy = fmaxf(0, B.joy - 0.25f); B.energy = 1; nPeaks = 0; tapAt = 0; }
  // vuruş
  if (linMag > 7 && !tapAt && rotMag < 150 && np <= 1) tapAt = now;
  if (linMag > 4) { gvx -= la[0] * 0.006f; gvy -= la[1] * 0.006f; }
}

// ================= ANA DÖNGÜ =================
static uint32_t lastMs = 0, lastLog = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(PIN_SDA, PIN_SCL, I2C_HZ);
  oledInit();
  mpuInit();
  bootMs = lastMs = millis();
  Serial.printf("Goz anahtarlik basladi. MPU6050: %s\n", mpuOk ? "OK" : "BULUNAMADI (kablolari kontrol et)");
}

static void handleSerial(uint32_t now) {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch >= '0' && ch <= '9') { forced = (Mood)(ch - '0'); hasForced = true; forcedUntil = now + 6000; moodUntil = 0; setMood(forced); }
    else if (ch == 'r') startRoll(now + 250);
    else if (ch == 'c') { for (int i = 0; i < 3; i++) base[i] = grav[i]; }
  }
}

void loop() {
  uint32_t now = millis();
  float dtm = fminf(50, now - lastMs), dt = dtm / 16.67f; lastMs = now;
  handleSerial(now);
  sensorStep(now, dtm);

  if (tapAt && now - tapAt > 280) {
    if (countPeaks(now) < 2) { B.energy = fminf(1, B.energy + 0.35f); B.joy = fminf(1, B.joy + 0.08f); trigger(SURPRISE, rnd(2200, 3200)); }
    tapAt = 0;
  }
  brainStep(now, dt / 60);
  bool drowsy = mood == NORMAL && B.energy < 0.35f;
  bool asleep = mood == SLEEP;

  // göz devirme
  float rollLid = 1;
  if (roll.on && now >= roll.t0) {
    float u = (now - roll.t0) / roll.dur;
    if (u < 1) { float e = u * u * (3 - 2 * u), th = PI_F * e; sacX = -cosf(th) * 0.9f * roll.dir; sacY = -0.25f - sinf(th) * 0.75f; scan.on = false; nextSac = now + 1e4; }
    else if (u < 1.5f) { sacX = 0.9f * roll.dir; sacY = -0.25f; }
    else if (u < 1.9f) { sacX = 0; sacY = 0.05f; }
    else { roll.on = false; nextSac = now + rnd(800, 1500); startBlink(now, false); }
    rollLid = u < 0.15f ? 1 - u / 0.15f * 0.4f : u < 1.6f ? 0.6f : fminf(1, 0.6f + (u - 1.6f) / 0.3f * 0.4f);
  }

  // etrafı inceleme
  moving *= powf(0.9f, dt);
  if (roll.on && now >= roll.t0) {}
  else if (scan.on) {
    float u = fminf(1, (now - scan.t0) / scan.dur), e = u * u * (3 - 2 * u);
    sacX = scan.x0 + (scan.x1 - scan.x0) * e; sacY = scan.y;
    if (u >= 1) { scan.on = false; nextSac = now + rnd(600, 1600); }
  } else if (now > nextSac && !asleep) {
    bool calm = moving < 0.15f; float slow = drowsy ? 2 : 1, r = frand();
    if (!calm) { sacX *= 0.3f; sacY = 0; nextSac = now + rnd(500, 1200); }
    else if (r < 0.25f) { sacX = 0; sacY = 0; nextSac = now + rnd(1500, 3500) * slow; }
    else if (r < 0.55f) { int d = frand() < 0.5f ? -1 : 1; scan = {true, sacX, d * rnd(0.6f, 0.9f), rnd(-0.2f, 0.15f), (float)now, rnd(1400, 2600) * slow}; }
    else if (r < 0.85f) { sacX = rnd(-0.85f, 0.85f); sacY = rnd(-0.4f, 0.3f); nextSac = now + rnd(700, 2200) * slow; }
    else if (r < 0.93f) { sacX = rnd(-0.4f, 0.4f); sacY = -0.65f; nextSac = now + rnd(1200, 2200); }
    else { sacX = rnd(-0.3f, 0.3f); sacY = 0.55f; nextSac = now + rnd(900, 1600); }
    if (!scan.on && hypotf(sacX - gzx, sacY - gzy) > 0.7f && frand() < 0.5f) startBlink(now, false);
  }
  driftX += (rnd(-1, 1) * 0.02f - driftX * 0.05f) * dt; driftY += (rnd(-1, 1) * 0.02f - driftY * 0.05f) * dt;
  float gx = asleep ? 0 : clampf(tiltX + sacX * (1 - fminf(1, fabsf(tiltX) * 1.5f)) + driftX, -1, 1);
  float gy = asleep ? 0 : clampf(tiltY + sacY * (1 - fminf(1, fabsf(tiltY) * 1.5f)) + driftY, -1, 1);
  const float K = 0.22f, D = 0.62f;
  gvx = (gvx + (gx - gzx) * K * dt) * powf(D, dt); gvy = (gvy + (gy - gzy) * K * dt) * powf(D, dt);
  gzx += gvx * dt; gzy += gvy * dt;

  // göz kırpma ve ifade geçişi
  if (now > nextBlink && !NOBLINK[mood] && !hasPending) startBlink(now, frand() < 0.18f);
  float b = blinkAmt(now);
  if (hasPending && now - blinkT >= 75) { mood = pending; hasPending = false; pop = 1; }
  pop *= powf(0.85f, dt);
  float open = (NOBLINK[mood] && !hasPending ? 1 : 1 - b) * (drowsy ? 0.55f + 0.2f * (B.energy / 0.35f) : 1);

  // çiz (64 satır için web sürümünden 2 px yukarı)
  memset(fb, 0, sizeof(fb));
  float breathe = sinf(now / (asleep ? 1500.0f : 1000.0f)) * (asleep ? 1.4f : 0.5f);
  float ox = gzx * 7, oy = gzy * (roll.on ? 10 : 4) + breathe, sc = 1 + pop * 0.12f;
  for (int s = -1; s <= 1; s += 2) {
    float persp = 1 + gzx * s * 0.08f;
    X = {64 + s * 29 + ox, 24 + oy, 1.9f * sc * persp, 1.9f * sc * persp * fmaxf(0.1f, open)};
    drawEye(mood, s, now);
    if (rollLid < 1) fillLocal(-14, -20, 14, -8 + (1 - rollLid) * 16, false, [](float, float) { return true; });
  }
  if (BLUSH[mood]) { blush(64 - 44 + ox, 45 + oy); blush(64 + 44 + ox, 45 + oy); }
  X = {64 + ox * 0.75f, 52 + oy * 0.85f, sc, sc};
  drawMouth(mood, now);
  if (asleep) { float t = fmodf(now / 1800.0f, 1); drawZ(94, (int)(14 - t * 5), 4); if (t > 0.35f) drawZ(101, (int)(6 - t * 5), 6); }
  oledFlush();

  if (now - lastLog > 300) {
    lastLog = now;
    Serial.printf("egim x %.2f y %.2f | ivme %.1f donus %.0f | mod %s | enerji %.2f keyif %.2f ofke %.2f bas %.2f sikilma %.2f%s\n",
                  tiltX, tiltY, linMag, rotMag, MOOD_NAME[mood], B.energy, B.joy, B.anger, B.dizzy, B.bored, faceDown ? " (ters)" : "");
  }
  // ~33 FPS
  uint32_t spent = millis() - now;
  if (spent < 30) delay(30 - spent);
}
