# Detector de pelotas por arcos (principal) + color — Orin / ZED

Prototipo offline, sin redes ni control del mecanismo. No validado en la cámara real
en competencia, solo en banco con dos pelotas (roja, verde) y luz fija.

El método PRINCIPAL es geometría — arcos/bordes alrededor de la circunferencia vía
Hough (`_detect_by_edges`), como se pidió desde el inicio del proyecto: centro, radio
y apoyo de borde. `_detect_by_hue` (color + circularidad del contorno) NO es un
segundo método de igual peso: es un respaldo que solo aporta candidatos en una zona
cuando los arcos no encontraron nada ahí en ese cuadro. Si los arcos ya ven algo en
la zona, el color no agrega ni reemplaza su resultado — solo lo clasifica (verde o
no) para decidir 1/0. Esto importaba en la práctica, no solo en principio: con los
dos métodos a igual peso, un cable rojo/naranja de la cámara (sin arco real, pura
coincidencia de color) se colaba como candidato en una zona vacía de arcos; con
arcos primero y color de respaldo, ese cable se evalúa igual, pero `min_radius` ya
lo descarta por tamaño (ver más abajo) — las dos correcciones trabajan juntas.

Por qué existe el respaldo de color y no se dejó solo con arcos: medido en banco,
una pelota roja en sombra fuerte nunca pasó de soporte de borde 0.54 (el umbral real
es 0.60) — sin importar cuánto se ajustaron los parámetros, el borde simplemente no
estaba ahí en escala de grises. Su matiz, en cambio, se mantuvo separado del fondo
incluso en esa sombra. Por qué el respaldo de color NO reemplaza a los arcos como
método principal: medido en banco, una pelota verde sobre el campo verde (mismo
matiz que el fondo) se fusionó en un solo blob con el fondo (circularidad 0.34, no
pasa el filtro) — ahí el color no ve nada útil, solo los arcos (soporte 0.62-0.73,
buen contraste de brillo) la encuentran. Ninguno de los dos "demuestra" por sí solo
que algo es pelota — un agujero circular puede pasar arcos, un cable del color
correcto puede pasar color. Por eso todo candidato se clasifica por color antes de
decidir si cuenta para arriba/abajo. Validar con video real antes de competencia:
todo esto se calibró con una sola pelota roja, una verde, y una foto de banco — no
con el set completo de colores ni con el montaje final.

## Preparación
Python 3, numpy y OpenCV (cv2). Usar el entorno ya funcional de la Orin;
no reemplazar su OpenCV solo para ejecutar este programa.

## Entrada ZED
Este programa usa OpenCV VideoCapture, no el SDK ZED (confirmado en el montaje real:
pyzed no está instalado en la Orin; el pipeline existente también abre la ZED como
UVC mediante cv2.VideoCapture, vía camera_select.open_bench). La ZED se resuelve por
/dev/video_zed (symlink estable a /dev/videoN; el número cambia entre arranques, el
symlink no). Entrega un único flujo YUYV con ambas vistas estéreo lado a lado
(1344x376 medido en la cámara real).

Dos modos para esa única entrada estéreo:
- --sbs-left: procesa SOLO la vista izquierda. Válido si a la distancia de montaje
  ambas hileras caben completas dentro de un solo ojo.
- --sbs-both: procesa las DOS mitades por separado y fusiona los candidatos de cada
  zona (arriba/abajo son el resultado combinado de ambos ojos). Necesario si a la
  distancia de montaje real una pelota puede quedar fuera del campo de un ojo y
  solo visible en el otro: confirmado en banco, una pelota visible únicamente en
  el ojo derecho no aparecía con --sbs-left y sí aparece con --sbs-both. Si esa
  vista cercana es la que tendrá el robot la mayoría del tiempo, usar --sbs-both,
  no --sbs-left.
NO usar ninguno de los dos flags si la fuente ya entrega una sola vista (p. ej. un
video ya recortado). No confundir las dos vistas estéreo (izquierda/derecha) con
las hileras superior e inferior (arriba/abajo): son dos particiones distintas de
la imagen. Para una cámara accesible solo por el SDK, se necesita adaptar la
adquisición; detect(frame, cfg) recibe una imagen BGR de una sola vista.

## Uso
    python detector.py --write-config config.json
    python detector.py --source /dev/video_zed --sbs-both --config config.json

/dev/video_zed es el symlink verificado en la Orin para la ZED 2 (ver cameras.json
en vision_orin, fuera de esta carpeta). Añadir --preview solo con escritorio y
OpenCV con GUI. Funciona sin imshow por SSH.
Para analizar una foto y guardar anotaciones:
    python detector.py --image foto.png --sbs-both --config config.json --save resultado.png
En una foto los campos arriba/abajo son null: se muestran candidatos, sin simular
confirmación temporal con repeticiones de la misma imagen.

## Ajuste geométrico obligatorio
regions contiene rectángulos [x0,y0,x1,y1] normalizados entre 0 y 1, en una sola
vista. Son zonas de centros admisibles, no recortes para calcular bordes. Modificar
cada zona de manera independiente para el desfase horizontal entre hileras. Las
zonas iniciales son ejemplos: deben ajustarse a la cámara instalada. Evitar que
un centro de agujero de base pueda entrar. Si eso no es posible, falta un filtro
específico de base o profundidad: no basta este prototipo.

min_radius y max_radius se expresan en píxeles DESPUÉS de redimensionar a width=640,
no en milímetros. Medir a las distancias de trabajo. Las coordenadas de salida
corresponden a esa misma imagen redimensionada. Radios admitidos demasiado amplios
incrementan falsos positivos. No se presupone alineación entre filas. Se aplican
igual a los dos métodos de detección (_detect_by_hue y _detect_by_edges).

min_radius=40 (subido de 12): las pelotas reales midieron 56-65px a la distancia de
banco actual, y un cable de la cámara medía 21px y pasaba como falso positivo del
respaldo de color — 40 lo excluye con margen a ambos lados. Esto es específico de
ESTA distancia de banco: el tamaño aparente de la pelota cambia con la distancia a
la cámara, así que hay que remedirlo y ajustar min_radius de nuevo con el montaje
final (más lejos, las pelotas se ven más chicas; más cerca, más grandes).

### Parámetros de _detect_by_edges (bordes/Hough)
hough_votes menor da más candidatos y falsos positivos. min_arc_support es la
fracción de 120 puntos del perímetro próxima a un borde; min_sectors exige apoyo
en sectores distintos (12 sectores). No son probabilidades ni garantía física.
CLAHE mejora contraste local, no crea bordes invisibles.

Medido con fotos reales de banco (no sintéticas): soporte de pelota real entre 0.617
y 0.725; un borde redondeado de un soporte/clip de montaje dio 0.65 (indistinguible
de una pelota real por soporte solo); un diente/ranura del separador dio entre 0.575
y 0.6, fluctuando por ruido entre capturas de la misma escena estática. min_arc_support
se subió de 0.55 a 0.60 porque eso quita el falso positivo del diente SIN perder la
pelota real medida — pero el soporte/clip (0.65) seguía sin poder separarse por
soporte solo. Resuelto por el filtro de color (siguiente sección), no por más ajuste
de este umbral: subirlo más seguía arriesgando perder pelotas reales débiles.

### Parámetros de _detect_by_hue (color + circularidad)
min_fill es el área real del contorno dividida entre el área del círculo que lo
envuelve (1.0 = círculo perfecto). Medido: pelota roja real 0.61-0.65 según el
umbral de brillo mínimo usado; un cable rojo/naranja de la cámara en el encuadre
de banco midió 0.57, peligrosamente cerca del umbral actual (0.5) — ese margen
angosto por sí solo no era confiable. Lo que lo excluye hoy es min_radius=40 (el
cable medía 21px): el cable nunca llega a evaluarse por fill porque ya se descarta
antes, por tamaño. min_fill=0.5 sigue angosto frente al 0.57 del cable si algún
día min_radius no alcanzara a filtrarlo (otro objeto rojo más grueso, por ejemplo)
— no se subió porque el margen con la pelota real (0.61+) también es angosto.

Se usa el cuadro delimitador del CONTORNO real para descartar blobs cortados por
el borde, no el círculo ajustado (minEnclosingCircle puede sobrepasar el borde por
imprecisión del ajuste aunque el blob completo esté dentro del cuadro — pasó con
la pelota roja real). Aun así, en una foto de banco la pelota roja no se detectó:
su blob de color llega exactamente a la última fila de píxeles del encuadre (cero
margen real, no un error de cálculo) — es un tema de encuadre de la cámara, igual
que el caso anterior con Hough, no algo que el software pueda inventar.

confirm_frames=3 exige continuidad aproximada del centro/radio en la misma fila.
Una ausencia reinicia inmediatamente la confirmación. max_gap_ms evita acumular
confirmaciones tras pausas. Ajustar con velocidad real (el robot se mueve y la
pelota entra por un lado del encuadre): un movimiento rápido puede reiniciar el
contador antes de que la pelota se asiente — es el comportamiento esperado, evita
confirmar una pelota que todavía está en tránsito. No calibrado con video en
movimiento real, solo con fotos fijas. La confirmación es presencia Y color
recogible (ver abajo), NO autorización de recogida — eso lo decide la Teensy.

## Color: ball_colors, y por qué hay dos rutas distintas que lo usan
ball_colors es un diccionario color→rangos de matiz (escala OpenCV H: 0-179; "rojo"
usa dos rangos porque el rojo cruza el cero del círculo de matiz). Se usa en los DOS
métodos, de forma distinta:

- `_detect_by_hue` lo usa para SEGMENTAR: construye una máscara binaria por color y
  busca contornos circulares dentro de ella. Es el método primario para el color.
- `_detect_by_edges` lo usa solo para CLASIFICAR, después de que Hough ya encontró
  un candidato por su borde: toma una muestra del interior y ve en qué rango de
  matiz cae. No segmenta nada, solo etiqueta.

min_ball_saturation=90 (saturación mínima para contar como "color de pelota", no
fondo/blanco/negro) se calibró con píxeles reales:

| región                          | saturación | matiz |
|----------------------------------|-----------:|------:|
| pelota roja                      | ~246       | ~2    |
| pelota verde                     | ~198       | ~75   |
| soporte/clip blanco (falso +)    | ~19        | —     |
| diente del separador (falso +)   | ~10–44     | —     |
| fondo del campo (parche bien iluminado) | ~11  | —     |
| fondo del campo EN SOMBRA        | ~183       | ~85   |

La última fila es la advertencia importante: el mismo campo, en sombra, mide una
saturación casi tan alta como una pelota real — por eso min_ball_saturation NO
alcanza para segmentar el fondo completo (confirmado: una máscara de saturación
sola cubrió 54% del cuadro, no solo la pelota). Separar por fondo requiere matiz
específico por color, no un umbral de saturación genérico — y aun así, cuando el
color de la pelota coincide con el del fondo (verde sobre campo verde), tampoco
el matiz alcanza: de ahí la necesidad de `_detect_by_edges` en paralelo.

ball_colors=[40,100] para verde es ancho a propósito (sesgado a atrapar de más, no
de menos) porque solo hay una pelota verde medida — más vale descartar por error
una pelota de otro color cercana al límite que dejar pasar una verde. Sin calibrar
contra el resto de colores de la temporada (el usuario confirmó que existen otros,
todos recogibles excepto el verde) — falta validar que ninguno caiga dentro de este
rango de matiz por error, sobre todo amarillo-verdoso o turquesa/cian. Un candidato
de un color NO configurado en ball_colors simplemente no se segmenta por matiz (pero
sigue pudiendo detectarse por _detect_by_edges, clasificado como "otro").

Un candidato verde SÍ aparece en el campo candidates del JSON (con "color":"verde"),
útil para depurar, pero nunca cuenta para arriba/abajo ni para confirm_frames: una
pelota verde nunca llega a reportarse como 1, sin importar cuántos frames seguidos
esté presente. "1" en arriba/abajo no significa solo "hay algo redondo", significa
"hay una pelota confirmada y recogible".

## Interfaz propuesta para Teensy (no conectada automáticamente)
Cada línea stdout es JSON con valid, seq, t_monotonic_ms, arriba, abajo y candidates.
arriba/abajo: 1 = candidato confirmado, con color de pelota Y no verde; 0 = no
confirmado, verde, o ningún candidato — en ningún caso es ausencia física demostrada
(puede haber una pelota verde presente y abajo/arriba reportar 0 a propósito). Pueden
ser 1 simultáneamente. No hay dependencia horizontal. valid=false indica fallo de
captura; fin de video también lo produce y termina. Un proceso bloqueado o muerto
puede no emitir nada: Teensy debe tener watchdog y rechazar datos antiguos. No
conservar el último 1 indefinidamente.

No se abre ningún puerto serie ni se acciona un motor. El protocolo real con Teensy
y su watchdog deben integrarse con el código existente. La Teensy conserva la lógica
de recogida (cuándo y cómo activar el servo); esta salida ya filtra verde por su
cuenta, pero sigue siendo responsabilidad de la Teensy decidir qué hacer con cada 1.

## Validación pendiente
Grabar desde la ZED instalada: ambas filas ocupadas, una vacía, ambas vacías,
agujeros visibles, verdes contra fondo verde, sombras, reflejos y movimiento real.
Un cable rojo/naranja de la propia cámara, visible en el banco de pruebas, pasó el
filtro de _detect_by_hue como candidato rojo (circularidad 0.57); se corrigió con
min_radius=40 (el cable medía 21px, muy por debajo de una pelota real), pero sigue
siendo buena práctica mantener el encuadre final libre de cables u objetos ajenos
al campo — min_radius ayuda, no sustituye un encuadre limpio.
Medir falsos positivos por fila, omisiones, latencia y duración de confirmación.
El desfase, las zonas, radios y rendimiento no se pueden certificar con estas fotos.
Referencia de Hough: https://docs.opencv.org/4.10.0/d4/d70/tutorial_hough_circle.html
