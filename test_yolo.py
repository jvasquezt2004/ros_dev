from ultralytics import YOLO
import cv2
import numpy as np
import urllib.request

# 1. Cargar modelo
print("Cargando modelo...")
model = YOLO("yolov8n.pt")

# 2. Descargar una imagen real (bus de Londres)
url = 'https://ultralytics.com/images/bus.jpg'
req = urllib.request.urlopen(url)
arr = np.asarray(bytearray(req.read()), dtype=np.uint8)
img = cv2.imdecode(arr, -1)

# 3. Detectar con confianza BAJISSIMA
print("Detectando...")
results = model(img, conf=0.1) # Confianza 10%

# 4. Mostrar resultados
print(f"Detectados: {len(results[0].boxes)} objetos")
res_plotted = results[0].plot()

cv2.imshow("Prueba YOLO", res_plotted)
cv2.waitKey(0)
cv2.destroyAllWindows()
