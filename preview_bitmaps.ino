/*
 * OpenFIRE Logo Bitmap 预览程序
 * 用于在串口监视器中预览位图数据
 */

#include "OpenFIRE_logo.h"

void setup() {
  Serial.begin(115200);
  while (!Serial); // 等待串口连接
  
  Serial.println("=== OpenFIRE Logo Bitmap 预览 ===");
  Serial.println();
  
  // 预览 customSplashBanner
  previewBitmap("customSplashBanner", customSplashBanner, CUSTSPLASHBANN_WIDTH, CUSTSPLASHBANN_HEIGHT);
  
  Serial.println();
  
  // 预览 customSplash  
  previewBitmap("customSplash", customSplash, CUSTSPLASH_WIDTH, CUSTSPLASH_HEIGHT);
  
  Serial.println();
  
  // 预览 logo_rgb (简化版，只显示基本信息)
  Serial.println("=== RGB Logo 信息 ===");
  Serial.print("尺寸: ");
  Serial.print(LOGO_RGB_WIDTH);
  Serial.print(" x ");
  Serial.println(LOGO_RGB_HEIGHT);
  Serial.print("数据大小: ");
  Serial.print(sizeof(logo_rgb));
  Serial.println(" 字节");
  
  Serial.println();
  
  // 预览 logo_rgb_alpha
  Serial.println("=== Alpha Logo 信息 ===");
  Serial.print("尺寸: ");
  Serial.print(LOGO_RGB_ALPHA_WIDTH);
  Serial.print(" x ");
  Serial.println(LOGO_RGB_ALPHA_HEIGHT);
  Serial.print("数据大小: ");
  Serial.print(sizeof(logo_rgb_alpha));
  Serial.println(" 字节");
  
  Serial.println();
  
  // 预览 logo_rgb_alpha_open
  Serial.println("=== Open Logo 信息 ===");
  Serial.print("尺寸: ");
  Serial.print(LOGO_RGB_ALPHA_OPEN_WIDTH);
  Serial.print(" x ");
  Serial.println(LOGO_RGB_ALPHA_OPEN_HEIGHT);
  Serial.print("数据大小: ");
  Serial.print(sizeof(logo_rgb_alpha_open));
  Serial.println(" 字节");
  
  Serial.println("=== 预览完成 ===");
}

void loop() {
  // 空循环
}

void previewBitmap(const char* name, const uint8_t* bitmap, int width, int height) {
  Serial.print("=== ");
  Serial.print(name);
  Serial.println(" ===");
  Serial.print("尺寸: ");
  Serial.print(width);
  Serial.print(" x ");
  Serial.println(height);
  Serial.print("数据大小: ");
  Serial.print(sizeof(bitmap[0]) * width * height / 8); // 估算大小
  Serial.println(" 字节");
  
  // 显示ASCII艺术预览
  Serial.println("ASCII预览:");
  printBitmapASCII(bitmap, width, height);
  Serial.println();
}

void printBitmapASCII(const uint8_t* bitmap, int width, int height) {
  int bytesPerRow = (width + 7) / 8; // 每行需要的字节数
  
  for (int y = 0; y < height && y < 32; y++) { // 限制显示高度避免串口溢出
    for (int x = 0; x < width && x < 64; x++) { // 限制显示宽度
      int byteIndex = (y * bytesPerRow) + (x / 8);
      int bitIndex = 7 - (x % 8);
      
      if (byteIndex < sizeof(bitmap)) {
        if (bitmap[byteIndex] & (1 << bitIndex)) {
          Serial.print("#"); // 像素为1
        } else {
          Serial.print("."); // 像素为0
        }
      }
    }
    Serial.println();
  }
  
  if (height > 32 || width > 64) {
    Serial.println("... (显示已截断)");
  }
}