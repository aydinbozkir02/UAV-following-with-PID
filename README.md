# PID Kontrollü YOLOv8 ile Nesne Takibi

## 📌 Proje Açıklaması

Bu proje, insansız hava araçlarında (İHA) kullanılmak üzere tasarlanmış, gerçek zamanlı nesne takibi yapabilen bir sistemdir. Sistem, YOLOv8 modeli ile tespit edilen hedefleri, PID kontrol algoritmaları kullanarak hassas bir şekilde takip eder ve hareket ederken hedef üzerinde kilitlenir.

## 🚀 Özellikler

- YOLOv8 modeli ile gerçek zamanlı nesne tespiti
- PID kontrol algoritmalarıyla stabil hedef takibi
- Kalman Filtresi ile konum tahmini ve stabilizasyon
- PID kontrol çıkışlarının anlık görsel gösterimi
- Kilitlenme durumunun süre bazlı kontrolü ve geri bildirimi

## 🛠 Kullanılan Teknolojiler

- Python
- OpenCV
- Ultralytics YOLOv8
- PID Kontrolör
- Kalman Filtresi
- Matplotlib (görselleştirme)

## 📁 Proje Yapısı

```
- PID.py (ana dosya, YOLOv8 ve PID entegrasyonu)
- pid_controller.py (PID algoritması)
- kalman_filter.py (Kalman filtresi algoritması)
- best.pt (YOLOv8 modeli)
- video.mp4 (örnek video)
```

## ⚙️ Kurulum ve Çalıştırma

### Bağımlılıkların Yüklenmesi:
```bash
pip install ultralytics opencv-python numpy matplotlib
```

### Çalıştırma:
```bash
python PID.py
```

## 🔧 Yapılandırma
- **PID Katsayıları**: PID hassasiyetini değiştirmek için `Kp`, `Ki`, `Kd` değerlerini PID.py dosyasında ayarlayabilirsiniz.
- **YOLO Modeli**: `best.pt` modelini kendi eğittiğiniz YOLOv8 modeli ile değiştirebilirsiniz.
- **Video Yolu**: Takip etmek istediğiniz videoyu `video.mp4` dosya yolu ile değiştirebilirsiniz.

## 📈 Görsel Çıktılar
- PID yönlendirme vektörü anlık olarak grafikle gösterilir.
- Takip edilen hedef üzerine görsel oklar çizilir ve hedefin kilitlenme süresi gösterilir.

## 📌 Kullanım Alanları
- İHA'larda otomatik hedef takibi
- Robotik görüş sistemleri
- Güvenlik ve izleme sistemleri

## 📝 Lisans
Bu proje MIT lisansı altındadır.
