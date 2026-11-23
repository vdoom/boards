"""Fills i18n/ins10studio_uk.ts with the Ukrainian texts below.

After changing interface strings:
    cmake --build <build dir> --target update_translations   # lupdate
    python tools/fill_translations.py                          # this
Strings without an entry here stay unfinished and are listed.
"""
from __future__ import annotations

import os
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
TS = os.path.normpath(os.path.join(HERE, "..", "i18n", "ins10studio_uk.ts"))

UK = {
    "N": "Пн",
    "E": "Сх",
    "S": "Пд",
    "W": "Зх",
    "gyro": "гіроскоп",
    "accelerometer": "акселерометр",
    "magnetometer": "магнітометр",
    "gyro temperature": "температура гіроскопа",
    "accelerometer temperature": "температура акселерометра",
    "Use the magnetometer ('set mag'; stored with the next save)":
        "Використовувати магнітометр ('set mag'; зберігається з наступним записом у flash)",
    "Every field of the board's calibration. Edit values (separated by spaces) and write them to RAM: the filter uses "
    "them at once. Nothing reaches flash until you save. Changed values are highlighted.":
        "Усі поля калібрування плати. Змініть значення (через пробіл) і запишіть у RAM: фільтр одразу їх використовує. "
        "У flash нічого не потрапляє, доки ви не збережете. Змінені значення підсвічено.",
    "Calibrated:": "Відкалібровано:",
    "Bit %1 (value %2) of the flags field": "Біт %1 (значення %2) поля flags",
    "Field": "Поле",
    "Values": "Значення",
    "Unit": "Одиниці",
    "Meaning": "Зміст",
    "Read from the board": "Прочитати з плати",
    "cal show": "cal show",
    "Write to RAM": "Записати в RAM",
    "Send the changed fields with 'cal set'": "Надіслати змінені поля командами 'cal set'",
    "Save to flash...": "Зберегти у flash...",
    "Write the changes, then 'cal save'": "Записати зміни, потім 'cal save'",
    "Discard changes": "Відкинути зміни",
    "Back to the calibration found when connecting (or last saved): the board's RAM and this table":
        "Повернути калібрування, знайдене під час підключення (або востаннє збережене): у RAM плати й у цій таблиці",
    "Erase...": "Стерти...",
    "cal erase: factory values, flash cleared, filter restarted":
        "cal erase: заводські значення, flash очищено, фільтр перезапущено",
    "Import...": "Імпорт...",
    "A JSON file from 'ins10.py cal export' or a saved 'cal show' listing; loaded into the table, not written yet":
        "Файл JSON з 'ins10.py cal export' або збережений вивід 'cal show'; завантажується в таблицю, на плату ще не пишеться",
    "Export JSON...": "Експорт JSON...",
    "The board's calibration as 'ins10.py cal export' writes it": "Калібрування плати так, як його пише 'ins10.py cal export'",
    "Export text...": "Експорт тексту...",
    "The board's calibration as 'cal show' lines": "Калібрування плати рядками 'cal show'",
    "Magnetometer use changed; save to flash to keep it after a restart.":
        "Використання магнітометра змінено; збережіть у flash, щоб це лишилося після перезапуску.",
    "%1: not a list of numbers": "%1: це не перелік чисел",
    "Read from the board.": "Прочитано з плати.",
    "Not written: %1": "Не записано: %1",
    "Nothing to write: the board already has these values.": "Нічого записувати: на платі вже ці значення.",
    "%1 field(s) written to RAM.": "Записано в RAM полів: %1.",
    "Save the board's calibration to flash? It then survives a restart.":
        "Зберегти калібрування плати у flash? Тоді воно переживе перезапуск.",
    "These changed fields are written first: %1.": "Спершу буде записано змінені поля: %1.",
    "Save to flash": "Збереження у flash",
    "Saved to flash.": "Збережено у flash.",
    "Back to the calibration found when connecting (or last saved).":
        "Повернуто калібрування, знайдене під час підключення (або востаннє збережене).",
    "Erase the calibration": "Стерти калібрування",
    "This erases the stored calibration, the learned thermal model and the settings in flash, puts the factory values in "
    "RAM and restarts the filter from initial alignment. It cannot be undone (export the calibration first if you may "
    "want it back).\n\nErase?":
        "Буде стерто збережене калібрування, вивчену температурну модель і налаштування у flash, у RAM потраплять "
        "заводські значення, фільтр перезапуститься з початкового вирівнювання. Скасувати це неможливо (спершу "
        "експортуйте калібрування, якщо воно може знадобитися).\n\nСтерти?",
    "Erased.": "Стерто.",
    "Import a calibration": "Імпорт калібрування",
    "Calibration (*.json *.txt *.cal);;All files (*)": "Калібрування (*.json *.txt *.cal);;Усі файли (*)",
    "Not imported: %1": "Не імпортовано: %1",
    "%1 field(s) loaded into the table; write them to RAM or save to flash to apply.":
        "У таблицю завантажено полів: %1. Щоб застосувати, запишіть їх у RAM або збережіть у flash.",
    "Export the calibration": "Експорт калібрування",
    "JSON (*.json)": "JSON (*.json)",
    "Written to %1": "Записано у %1",
    "Text (*.txt)": "Текст (*.txt)",
    "cancelled": "скасовано",
    "only %1 distinct orientations found; at least 6 are needed (each face of the board up), 12 or more are better":
        "знайдено лише %1 різних орієнтацій; потрібно щонайменше 6 (кожною гранню догори), краще 12 і більше",
    "accelerometer calibration is far outside the sensor's tolerance; were the rest positions really at rest?":
        "калібрування акселерометра далеко за допуском сенсора; чи справді плата лежала нерухомо в положеннях спокою?",
    "found %1 rest positions; at least 6 are needed. Leave the board untouched for 3 s in each position.":
        "знайдено %1 положень спокою; потрібно щонайменше 6. Лишайте плату нерухомою на 3 с у кожному положенні.",
    "%1 distinct orientations: accelerometer scale and bias were fitted, cross-axis terms were not (needs 9 or more).":
        "%1 різних орієнтацій: визначено масштаб і зсув акселерометра, перехресні зв'язки — ні (потрібно 9 і більше).",
    "gyro scale factors were not fitted: too few usable turns between rest positions, or too little variety of rotation axes.":
        "масштаби гіроскопа не визначено: замало придатних поворотів між положеннями спокою або замало різних осей обертання.",
    "gyro scale factors were fitted, misalignment was not (needs more turns about different axes).":
        "масштаби гіроскопа визначено, неортогональність — ні (потрібно більше поворотів навколо різних осей).",
    "accelerometer residual %1 mg is high: was the board moving or vibrating in some positions?":
        "нев'язка акселерометра %1 мг велика: чи не рухалася або не вібрувала плата в деяких положеннях?",
    "magnetometer data has no spread": "дані магнітометра не мають розкиду",
    "the samples do not describe an ellipsoid: rotate the board through many more orientations":
        "вибірки не описують еліпсоїд: проверніть плату через значно більше орієнтацій",
    "degenerate ellipsoid fit": "вироджений еліпсоїд",
    "only %1 magnetometer samples; record at least 20 s": "лише %1 вибірок магнітометра; записуйте щонайменше 20 с",
    "soft-iron matrix is not positive definite; record again": "матриця soft iron не додатно визначена; запишіть ще раз",
    "only %1 % of all directions were sampled; the soft-iron estimate is weak. Rotate about all three axes, including upside down.":
        "охоплено лише %1 % усіх напрямків; оцінка soft iron слабка. Обертайте навколо всіх трьох осей, зокрема догори дном.",
    "field magnitude %1 uT is outside the earth's range (22-67 uT): strong magnetic material nearby?":
        "модуль поля %1 мкТл поза межами земного (22–67 мкТл): поруч сильний магнітний матеріал?",
    "residual %1 % is high: the field was not uniform (moving near steel, cables, magnets) or the sensor is saturating.":
        "розкид %1 % великий: поле було неоднорідним (рух біля сталі, кабелів, магнітів) або сенсор насичується.",
    "too few samples in slow, smooth motion to align the magnetometer to the IMU: turn the board more slowly.":
        "замало вибірок у повільному плавному русі, щоб вирівняти магнітометр до IMU: повертайте плату повільніше.",
    "the alignment to the IMU is poorly determined by this recording; it was not applied.":
        "цей запис погано визначає вирівнювання до IMU; його не застосовано.",
    "dip angle varies by %1 deg over the recording: motion too fast or field not uniform.":
        "нахилення змінюється на %1° за запис: рух надто швидкий або поле неоднорідне.",
    "recording too short": "запис закороткий",
    "temperature changed by only %1 C; at least 3 C is needed, 15 C or more is better (start from a cold board)":
        "температура змінилася лише на %1 °C; потрібно щонайменше 3 °C, краще 15 °C і більше (почніть із холодної плати)",
    "temperature span %1 C: fitted a polynomial of order %2. Outside the recorded range the compensation is an extrapolation.":
        "діапазон температури %1 °C: підібрано поліном порядку %2. Поза записаним діапазоном компенсація — це екстраполяція.",
    "the board moved during the recording; those samples were rejected.":
        "плата рухалася під час запису; ці вибірки відкинуто.",
    "the accelerometer is not calibrated yet. The magnetometer is aligned to its axes, so its bias (up to about a degree) "
    "goes into the alignment. Calibrate the accelerometer first, then the magnetometer again.":
        "акселерометр ще не відкалібровано. Магнітометр вирівнюється до його осей, тож зсув акселерометра (до градуса) "
        "потрапить у вирівнювання. Спершу відкалібруйте акселерометр, потім магнітометр ще раз.",
    "the magnetometer's axes are a mirror image of the accelerometer's, so the alignment is a reflection rather than a "
    "rotation (as on the INS10-A3). The calibration includes the reflection; the heading is right if the hemisphere "
    "setting is.":
        "осі магнітометра — дзеркальне відображення осей акселерометра, тож вирівнювання є відображенням, а не поворотом "
        "(як на INS10-A3). Калібрування враховує відображення; курс правильний, якщо правильно вказано півкулю.",
    "the dip angle is negative, which is right only in the southern hemisphere. In the north it means one sensor's axes "
    "are mirrored; check the wiring and driver before trusting the heading.":
        "нахилення від'ємне, що правильно лише в південній півкулі. У північній це означає дзеркальні осі одного із "
        "сенсорів; перевірте з'єднання й драйвер, перш ніж довіряти курсу.",
    "The result was not aligned to the IMU frame and will not be written.":
        "Результат не вирівняно до осей IMU, тож його не буде записано.",
    "Which calibrations are present (bit mask)": "Які калібрування є (бітова маска)",
    "Gyro bias at the reference temperature": "Зсув гіроскопа при опорній температурі",
    "Gyro scale and misalignment, by rows": "Масштаб і неортогональність гіроскопа, по рядках",
    "Accelerometer bias": "Зсув акселерометра",
    "Accelerometer scale and cross-axis terms": "Масштаб і перехресні зв'язки акселерометра",
    "Magnetometer hard iron": "Hard iron магнітометра",
    "Soft iron and rotation into the IMU axes": "Soft iron і поворот в осі IMU",
    "Reference temperature of the gyro polynomial": "Опорна температура полінома гіроскопа",
    "Gyro X bias drift: terms in dT, dT^2, dT^3": "Дрейф зсуву гіроскопа X: члени dT, dT², dT³",
    "Gyro Y bias drift: terms in dT, dT^2, dT^3": "Дрейф зсуву гіроскопа Y: члени dT, dT², dT³",
    "Gyro Z bias drift: terms in dT, dT^2, dT^3": "Дрейф зсуву гіроскопа Z: члени dT, dT², dT³",
    "Reference temperature of the accelerometer polynomial": "Опорна температура полінома акселерометра",
    "Accelerometer X bias drift: terms in dT, dT^2, dT^3": "Дрейф зсуву акселерометра X: члени dT, dT², dT³",
    "Accelerometer Y bias drift: terms in dT, dT^2, dT^3": "Дрейф зсуву акселерометра Y: члени dT, dT², dT³",
    "Accelerometer Z bias drift: terms in dT, dT^2, dT^3": "Дрейф зсуву акселерометра Z: члени dT, dT², dT³",
    "Magnetic declination, east positive (0: heading from magnetic north)":
        "Магнітне схилення, на схід додатне (0: курс від магнітної півночі)",
    "Field magnitude found by the calibration (informational)": "Модуль поля, знайдений калібруванням (довідково)",
    "Dip angle found by the calibration (informational)": "Нахилення, знайдене калібруванням (довідково)",
    "unknown field '%1'": "невідоме поле '%1'",
    "%1 takes %2 values, not %3": "%1 має %2 значень, а не %3",
    "%1: values must be finite numbers": "%1: значення мають бути скінченними числами",
    "flags must be a whole number from 0 to 255": "flags має бути цілим числом від 0 до 255",
    "not a JSON object: %1": "це не об'єкт JSON: %1",
    "%1: expected a list of numbers": "%1: очікується перелік чисел",
    "no calibration found in the file": "у файлі немає калібрування",
    "Recommended order: accelerometer and gyro first, then the magnetometer (it is aligned to the accelerometer's axes), "
    "then, if wanted, temperature.":
        "Рекомендований порядок: спершу акселерометр і гіроскоп, потім магнітометр (він вирівнюється до осей "
        "акселерометра), потім за бажання температура.",
    "Fields": "Поля",
    "Accelerometer and gyro": "Акселерометр і гіроскоп",
    "Magnetometer": "Магнітометр",
    "Temperature": "Температура",
    "Connect a board first, or load a recording from a file.": "Спершу підключіть плату або завантажте запис із файлу.",
    "Could not read the calibration: %1": "Не вдалося прочитати калібрування: %1",
    "The connection ended during the recording; what was recorded so far is kept.":
        "З'єднання обірвалося під час запису; записане досі збережено.",
    "a recording is in progress": "триває запис",
    "no $RAW or $MAG records in the file": "у файлі немає записів $RAW чи $MAG",
    "nothing recorded": "нічого не записано",
    "not connected": "не підключено",
    "Written to the board's RAM; the filter uses it now.": "Записано в RAM плати; фільтр уже це використовує.",
    "Written and saved to flash.": "Записано й збережено у flash.",
    "The calibration from before the run is back in the board's RAM.": "У RAM плати повернуто калібрування, що було до запуску.",
    "Compass heading of the board's X axis. Grey: not referenced to the magnetic field (magnetometer not calibrated or "
    "disabled), so it is relative to the heading at power-up.":
        "Компасний курс осі X плати. Сірий: курс не прив'язано до магнітного поля (магнітометр не відкалібровано або "
        "вимкнено), тож він відлічується від курсу на момент увімкнення.",
    "no heading": "курсу немає",
    "Serial port. Raspberry Pi Pico boards (USB vendor 0x2E8A) are listed first.":
        "Послідовний порт. Плати Raspberry Pi Pico (USB vendor 0x2E8A) показано першими.",
    "Look for serial ports again": "Знову пошукати послідовні порти",
    "Connect": "Підключити",
    "Lines that arrived truncated, run together or unrecognised. The firmware drops output when the PC does not read fast enough.":
        "Рядки, що надійшли обрізаними, склеєними чи нерозпізнаними. Прошивка відкидає вивід, якщо ПК не встигає читати.",
    "IMU samples missing between $RAW lines (from the sample numbers).":
        "Вибірки IMU, яких бракує між рядками $RAW (за номерами вибірок).",
    "Simulated board (demo)": "Імітатор плати (демо)",
    "Disconnect": "Відключити",
    "no board": "плати немає",
    "clock external": "такт зовнішній",
    "clock %1": "такт %1",
    "fw %1 · %2 · %3": "прошивка %1 · %2 · %3",
    "damaged lines: %1": "зіпсовані рядки: %1",
    "lost samples: %1": "втрачені вибірки: %1",
    "Hide streams": "Сховати потоки",
    "Command, e.g. help, info, status, cal show": "Команда, наприклад help, info, status, cal show",
    "Send": "Надіслати",
    "Clear": "Очистити",
    "Do not show $ATT, $RAW, $MAG and $BARO records": "Не показувати записи $ATT, $RAW, $MAG і $BARO",
    "Commands typed here go to the board as they are. A 'stream att' rate set here becomes the board's own rate, the one "
    "'cal save' stores.":
        "Набрані тут команди йдуть на плату як є. Частота 'stream att', задана тут, стає власною частотою плати — тією, "
        "яку зберігає 'cal save'.",
    "%1: %2": "%1: %2",
    "Disconnected": "Відключено",
    "Opening port": "Відкриття порту",
    "Identifying firmware": "Визначення прошивки",
    "Connected": "Підключено",
    "Not the AHRS firmware": "Це не прошивка AHRS",
    "Restoring board state": "Відновлення стану плати",
    "Rebooting": "Перезавантаження",
    "Connection lost": "З'єднання втрачено",
    "no reply within %1 s (is this the AHRS firmware?)": "немає відповіді за %1 с (чи це прошивка AHRS?)",
    "cannot use port %1": "не вдається використати порт %1",
    "cannot open %1: %2": "не вдається відкрити %1: %2",
    "'info' did not report fw=ins10-ahrs": "'info' не повідомив fw=ins10-ahrs",
    "This is not the ins10_ahrs firmware (%1). The bench test ins10_test, for one, does not speak this protocol; the "
    "console still works.":
        "Це не прошивка ins10_ahrs (%1). Стендовий тест ins10_test, наприклад, цього протоколу не розуміє; консоль "
        "працює й так.",
    "empty command": "порожня команда",
    "command longer than the firmware accepts (%1 characters, %2 words)":
        "команда довша, ніж приймає прошивка (%1 символів, %2 слів)",
    "write to the port failed": "запис у порт не вдався",
    "disconnecting": "відключення",
    "disconnected": "відключено",
    "the board's stream settings could not all be restored": "не всі налаштування потоків плати вдалося відновити",
    "connection lost: %1": "з'єднання втрачено: %1",
    "the board is in the USB bootloader": "плата в USB-завантажувачі",
    "the board did not come back after the reboot": "плата не з'явилася після перезавантаження",
    "error": "помилка",
    "Waiting for 'info' from the board.": "Очікування 'info' від плати.",
    "Waiting for 'status' from the board.": "Очікування 'status' від плати.",
    "IMU running at 512 Hz": "IMU працює на 512 Гц",
    "The ICM-42688-P did not start. Run the bench test (ins10_test) and check the SPI wiring and the IMU supply.":
        "ICM-42688-P не запустився. Запустіть стендовий тест (ins10_test) і перевірте з'єднання SPI та живлення IMU.",
    "The output data rate should be 512.000 Hz; the driver configures it, so another value means the configuration did not take.":
        "Частота вибірок має бути 512,000 Гц; її задає драйвер, тож інше значення означає, що налаштування не застосувалося.",
    "IMU clock from the 32.768 kHz oscillator": "Такт IMU від генератора 32,768 кГц",
    "clock=internal: the 32.768 kHz clock (CLKIN) was not picked up. The filter works, but on the measured rate. Check the "
    "oscillator and the CLKIN pin.":
        "clock=internal: такт 32,768 кГц (CLKIN) не підхопився. Фільтр працює, але з виміряною частотою. Перевірте "
        "генератор і вивід CLKIN.",
    "Magnetometer and barometer found": "Магнітометр і барометр знайдено",
    "A sensor did not answer. The bench test (ins10_test) checks each chip on its own.":
        "Сенсор не відповів. Стендовий тест (ins10_test) перевіряє кожну мікросхему окремо.",
    "The magnetometer is switched off ('set mag 0'); heading is not referenced to north.":
        "Магнітометр вимкнено ('set mag 0'); курс не прив'язано до півночі.",
    "Accelerometer reads 1 g at rest": "Акселерометр у спокої показує 1 g",
    "accel %1 %2 %3 g, norm %4 g": "прискорення %1 %2 %3 g, норма %4 g",
    "The norm is off by a factor of four: the scale of the 20-bit FIFO format in drivers/icm42688.h is wrong.":
        "Норма відрізняється вчетверо: хибний масштаб 20-бітного формату FIFO у drivers/icm42688.h.",
    "Leave the board still, components up, for a few seconds.": "Залиште плату нерухомою компонентами догори на кілька секунд.",
    "The filter has not reported 'still' for %1 s. Either something disturbs the board (knocks on the table, a pulled "
    "cable, a hand), or the firmware's stillness thresholds are too tight for this board. To tell which, put the board "
    "on a solid surface and record 30-60 s of raw data on the Recording page (raw 1).":
        "Фільтр уже %1 с не повідомляє про спокій ('still'). Або плату щось турбує (поштовхи столу, натягнутий кабель, "
        "рука), або пороги спокою в прошивці затісні для цієї плати. Щоб розрізнити, покладіть плату на тверду поверхню "
        "й запишіть 30–60 с сирих даних на сторінці «Запис» (raw 1).",
    "At rest the norm should be within 0.98-1.02 g. A small error goes away with the accelerometer calibration; a large "
    "one points at the driver's scale.":
        "У спокої норма має бути в межах 0,98–1,02 g. Невелику похибку прибере калібрування акселерометра; велика вказує "
        "на масштаб у драйвері.",
    "Gyro bias below 1 deg/s, filter aligned and still": "Зсув гіроскопа менше 1 °/с, фільтр вирівняний і в спокої",
    "gyro_bias %1 %2 %3 dps; flags %4": "gyro_bias %1 %2 %3 °/с; прапорці %4",
    "Waiting for the flags 'aligned still': leave the board untouched.":
        "Очікування прапорців 'aligned still': не торкайтеся плати.",
    "A bias above 1 deg/s is unusual for the ICM-42688-P: was the board really still, or is the gyro scale wrong?":
        "Зсув понад 1 °/с незвичний для ICM-42688-P: чи справді плата лежала нерухомо, чи не хибний масштаб гіроскопа?",
    "No IMU gaps, overflows, bad packets or bus errors": "IMU без розривів, переповнень, зіпсованих пакетів і помилок шини",
    "The counters are growing: the FIFO is not read in time, or the SPI link is unreliable (wiring length, clock speed).":
        "Лічильники ростуть: FIFO не встигають читати або лінія SPI ненадійна (довжина дротів, частота такту).",
    "Not zero, but not growing either: something happened earlier (at power-up?). Watch whether they grow.":
        "Не нуль, але й не ростуть: щось сталося раніше (під час увімкнення?). Стежте, чи не ростуть.",
    "Processor keeps up (load, largest batch)": "Процесор встигає (завантаження, найбільший пакет)",
    "imu+filter %1 % of CPU, largest batch %2 samples": "IMU і фільтр %1 % процесора, найбільший пакет %2 вибірок",
    "The largest batch stays above 10 samples: the main loop does not keep up with the IMU.":
        "Найбільший пакет стабільно понад 10 вибірок: головний цикл не встигає за IMU.",
    "The filter takes most of the processor; on the RP2040 (no FPU) 10-25 % is expected.":
        "Фільтр забирає більшу частину процесора; на RP2040 (без FPU) очікується 10–25 %.",
    "Higher than the 10-25 % expected on a Pico 1; little headroom is left.":
        "Більше за очікувані на Pico 1 10–25 %; запасу лишилося мало.",
    "Magnetometer: ~50 samples/s, ~1 SET/RESET pair/s": "Магнітометр: ~50 вибірок/с, ~1 пара SET/RESET за секунду",
    "Measuring the rates over several seconds...": "Вимірювання частот за кілька секунд...",
    "%1 samples/s, %2 pairs/s, errors +%3": "%1 вибірок/с, %2 пар/с, помилок +%3",
    "Magnetometer errors are growing: check its SPI chip select and supply.":
        "Помилки магнітометра ростуть: перевірте його вибір мікросхеми SPI і живлення.",
    "No magnetometer samples arrive.": "Вибірки магнітометра не надходять.",
    "The rates differ from the expected ~50/s and ~1/s: the measurement state machine is not running as designed.":
        "Частоти відрізняються від очікуваних ~50/с і ~1/с: автомат вимірювань працює не так, як задумано.",
    "Last restart was normal (no watchdog)": "Останній перезапуск штатний (не сторож)",
    "The watchdog restarted the board: the main loop hung. If this repeats, save a log of what was happening.":
        "Плату перезапустив сторож: головний цикл зависав. Якщо повториться, збережіть журнал того, що відбувалося.",
    "Calibration stored in flash": "Калібрування збережене у flash",
    "Nothing is stored yet. After 'Save to flash' and a reboot, 'info' should show stored=1; if it does not, writing the "
    "flash failed.":
        "Поки нічого не збережено. Після «Зберегти у flash» і перезавантаження 'info' має показати stored=1; якщо ні — "
        "запис у flash не вдався.",
    "<b>First run on the board.</b> These checks show whether the module is soldered and wired correctly and whether "
    "its sensors, clock and data stream work. Lay the board still, components up, and leave this page open for about "
    "half a minute. If something fails, the details tell what to look at; the raw replies are in the Console page.":
        "<b>Перший запуск на платі.</b> Ці перевірки показують, чи правильно модуль припаяно й підключено і чи працюють "
        "його сенсори, тактування й потік даних. Покладіть плату нерухомо компонентами догори і тримайте цю сторінку "
        "відкритою приблизно пів хвилини. Якщо щось не так, у подробицях сказано, що перевірити; сирі відповіді — на "
        "сторінці «Консоль».",
    "Check": "Перевірка",
    "Result": "Результат",
    "Measured": "Виміряно",
    "Read info again": "Прочитати info ще раз",
    "Copy report": "Копіювати звіт",
    "Copy the checklist with the measured values as text": "Скопіювати перелік перевірок із виміряними значеннями як текст",
    "Connect a board to run the checks.": "Підключіть плату, щоб виконати перевірки.",
    "%1 passed, %2 need attention, %3 failed, %4 still waiting.":
        "Гаразд: %1, увага: %2, збій: %3, ще очікують: %4.",
    "INS10 first-run checks": "Перевірки першого запуску INS10",
    "OpenGL 3.3 is not available here (got %1.%2). The 3D view is off; everything else works.":
        "OpenGL 3.3 тут недоступний (отримано %1.%2). Тривимірний вигляд вимкнено; решта працює.",
    "The 3D view's shaders did not compile: %1": "Шейдери тривимірного вигляду не скомпілювалися: %1",
    "Starting OpenGL...": "Запуск OpenGL...",
    "<b>Accelerometer and gyro: multi-position calibration.</b> Put the board down in 12 or more clearly different "
    "orientations: each face and edge down, and a few tilted ones (a small box to lean it against helps). Leave it "
    "completely still for 3-4 s in each, then turn it to the next in one smooth motion; the turns calibrate the gyro, "
    "so vary the direction of turning. Rest positions are listed as they are recognised.":
        "<b>Акселерометр і гіроскоп: калібрування в багатьох положеннях.</b> Покладіть плату у 12 або більше помітно "
        "різних орієнтацій: кожною гранню й ребром донизу, а також кілька похилих (зручно спирати на невелику коробку). "
        "У кожній залиште її повністю нерухомою на 3–4 с, потім одним плавним рухом переведіть у наступну; повороти "
        "калібрують гіроскоп, тож змінюйте напрямок обертання. Положення спокою з'являються в переліку, щойно їх "
        "розпізнано.",
    "Direction of gravity in the board's axes for every rest position recognised":
        "Напрямок гравітації в осях плати для кожного розпізнаного положення спокою",
    "#": "№",
    "Duration, s": "Тривалість, с",
    "Gravity in board axes, g": "Гравітація в осях плати, g",
    "Temp, C": "Темп., °C",
    "Progress": "Хід",
    "Rest positions": "Положення спокою",
    "Distinct orientations": "Різні орієнтації",
    "At least 6 orientations are needed, 9 for the cross-axis terms; 12 or more are better.":
        "Потрібно щонайменше 6 орієнтацій, 9 — для перехресних зв'язків; краще 12 і більше.",
    "Need at least 6 (each face of the board down).": "Потрібно щонайменше 6 (кожною гранню плати донизу).",
    "Enough for bias and scale; 9 or more also give the cross-axis terms.":
        "Досить для зсуву й масштабу; 9 і більше дадуть і перехресні зв'язки.",
    "Enough for the full matrix; a few more improve it (12+).": "Досить для повної матриці; ще кілька покращать її (12+).",
    "Plenty. Stop when you like.": "Удосталь. Зупиняйте, коли зручно.",
    "Rest positions, orientations": "Положень спокою, орієнтацій",
    "Accelerometer: |a| error over the positions": "Акселерометр: похибка |a| по положеннях",
    "%1 mg before, %2 mg after": "%1 мг до, %2 мг після",
    "Accelerometer bias, mg": "Зсув акселерометра, мг",
    "Accelerometer matrix": "Матриця акселерометра",
    "Gyro bias, deg/s": "Зсув гіроскопа, °/с",
    "Gyro: gravity mismatch after the turns": "Гіроскоп: розбіжність напрямку гравітації після поворотів",
    "%1 deg before, %2 deg after (%3 matrix)": "%1° до, %2° після (матриця %3)",
    "Gyro matrix": "Матриця гіроскопа",
    "not fitted; the gyro flag is not set": "не визначено; прапорець гіроскопа не встановлюється",
    "write failed": "запис не вдався",
    "Southern hemisphere (the field points up)": "Південна півкуля (поле спрямоване вгору)",
    "Which way the earth's field points decides whether the magnetometer is aligned to the IMU with a rotation or with "
    "a mirror image of its axes. Leave it unticked north of the magnetic equator, which includes all of Europe and North "
    "America.":
        "Від того, куди спрямоване поле Землі, залежить, чи магнітометр вирівнюється до IMU поворотом, чи дзеркальним "
        "відображенням осей. На північ від магнітного екватора (уся Європа й Північна Америка) лишайте без позначки.",
    "<b>Magnetometer: hard and soft iron, and alignment to the IMU axes.</b> Do this with the board mounted as it will be "
    "used (its carrier, wires and nearby parts are part of what is calibrated), away from steel furniture, speakers, "
    "monitors and power cables. Calibrate the accelerometer first. Turn the board slowly through every orientation: "
    "several full turns about each of its three axes, including upside down; about 60-90 s, slower than one turn in 6 s, "
    "without shaking. Aim for 80 % or more of the cells green.":
        "<b>Магнітометр: hard і soft iron та вирівнювання до осей IMU.</b> Робіть це з платою, змонтованою так, як вона "
        "працюватиме (носій, дроти й сусідні деталі входять у те, що калібрується), подалі від сталевих меблів, колонок, "
        "моніторів і силових кабелів. Спершу відкалібруйте акселерометр. Повільно прокрутіть плату через усі орієнтації: "
        "кілька повних обертів навколо кожної з трьох осей, зокрема догори дном; 60–90 с, не швидше одного оберту за "
        "6 с, без струшування. Цільте у 80 % і більше зелених клітинок.",
    "Raw magnetometer samples around the centre of their range, scaled to the unit sphere. Cells turn green once a sample "
    "points through them.":
        "Сирі вибірки магнітометра навколо середини їхнього діапазону, приведені до одиничної сфери. Клітинка зеленіє, "
        "щойно крізь неї пройде вибірка.",
    "Directions covered (aim for 80 % or more)": "Охоплено напрямків (ціль — 80 % і більше)",
    "no samples yet": "вибірок ще немає",
    "%1 magnetometer samples": "Вибірок магнітометра: %1",
    "<b>Slower!</b> Turning at %1 deg/s; keep below 60.": "<b>Повільніше!</b> Обертання %1 °/с; тримайте нижче 60.",
    "Turning at %1 deg/s": "Обертання %1 °/с",
    "Grey: the samples before calibration; green: after, on the unit sphere.":
        "Сірі — вибірки до калібрування; зелені — після, на одиничній сфері.",
    "Samples": "Вибірки",
    "Directions covered": "Охоплено напрямків",
    "Hard iron, uT": "Hard iron, мкТл",
    "Field magnitude": "Модуль поля",
    "%1 uT (22-67 on the earth; the value for your place is in the NOAA WMM calculator)":
        "%1 мкТл (на Землі 22–67; значення для вашого місця — у калькуляторі NOAA WMM)",
    "Spread after calibration": "Розкид після калібрування",
    "%1 % (below 1 % is good)": "%1 % (менше 1 % — добре)",
    "Aligned to the IMU axes": "Вирівняно до осей IMU",
    "yes": "так",
    "yes, with a mirror image (the magnetometer's axes are mirrored)":
        "так, дзеркальним відображенням (осі магнітометра дзеркальні)",
    "Dip angle": "Нахилення",
    "%1 deg, consistent to %2 deg (grows with latitude: about +65 at 50 deg N, +75 at 60 deg N; negative south of the "
    "magnetic equator)":
        "%1°, сталість %2° (зростає з широтою: близько +65 на 50° пн. ш., +75 на 60° пн. ш.; на південь від магнітного "
        "екватора від'ємне)",
    "no": "ні",
    "Matrix (soft iron and alignment)": "Матриця (soft iron і вирівнювання)",
    "INS10 Studio": "INS10 Studio",
    "Connection": "З'єднання",
    "Overview": "Огляд",
    "Plots": "Графіки",
    "First run": "Перший запуск",
    "Status": "Стан",
    "Calibration": "Калібрування",
    "Recording": "Запис",
    "Console": "Консоль",
    "&File": "&Файл",
    "&Settings...": "&Налаштування...",
    "&Quit": "&Вийти",
    "&Board": "&Плата",
    "Connect the &simulated board": "Підключити &імітатор плати",
    "&Disconnect": "&Відключити",
    "Restart the &filter...": "Перезапустити &фільтр...",
    "Restart the filter": "Перезапуск фільтра",
    "The filter starts again from initial alignment. Leave the board still for 2 s. Restart?":
        "Фільтр почне знову з початкового вирівнювання. Залиште плату нерухомою на 2 с. Перезапустити?",
    "&Reboot the Pico...": "Пере&завантажити Pico...",
    "Reboot": "Перезавантаження",
    "The Pico restarts; the application reconnects when the port comes back. Unsaved calibration changes are lost. Reboot?":
        "Pico перезапуститься; застосунок підключиться знову, коли порт з'явиться. Незбережені зміни калібрування "
        "пропадуть. Перезавантажити?",
    "Enter the USB &bootloader...": "Перейти в USB-&завантажувач...",
    "Bootloader": "Завантажувач",
    "The Pico switches to its USB bootloader (BOOTSEL) for flashing new firmware and the port disappears. Continue?":
        "Pico перейде у свій USB-завантажувач (BOOTSEL) для прошивання, і порт зникне. Продовжити?",
    "Simulated &board": "&Імітатор плати",
    "Demo choreography": "Демонстраційний рух",
    "Slow tumble (for the magnetometer wizard)": "Повільне перекидання (для майстра магнітометра)",
    "Rest positions (for the accelerometer and gyro wizard)": "Положення спокою (для майстра акселерометра й гіроскопа)",
    "Lying still, warming up (for the temperature wizard)": "Лежить нерухомо, прогрівається (для майстра температури)",
    "Time x%1": "Час ×%1",
    "&Help": "&Довідка",
    "&About": "&Про програму",
    "About INS10 Studio": "Про INS10 Studio",
    "<b>INS10 Studio %1</b><p>Orientation display, diagnostics and calibration for the INS10-A3 board running the "
    "ins10_ahrs firmware.</p><p>Qt %2. Linear algebra: Eigen.</p>":
        "<b>INS10 Studio %1</b><p>Показ орієнтації, діагностика й калібрування плати INS10-A3 з прошивкою "
        "ins10_ahrs.</p><p>Qt %2. Лінійна алгебра: Eigen.</p>",
    "The board refused: %1": "Плата відмовила: %1",
    "Restoring the board's settings...": "Відновлення налаштувань плати...",
    "View from above": "Вигляд згори",
    "Default view": "Типовий вигляд",
    "Drag to turn the view, wheel to zoom, double-click to reset.":
        "Тягніть, щоб повернути вигляд, коліщатком — масштаб, подвійний клік — скинути.",
    "Roll": "Крен",
    "Rotation about the body X axis, deg": "Поворот навколо осі X плати, °",
    "Pitch": "Тангаж",
    "Rotation about the body Y axis, deg": "Поворот навколо осі Y плати, °",
    "Yaw": "Рискання",
    "Counter-clockwise from east seen from above, deg; zero when X points east":
        "Проти годинникової стрілки від сходу, якщо дивитися згори, °; нуль, коли X дивиться на схід",
    "Heading": "Курс",
    "Compass heading of the X axis, clockwise from north, deg": "Компасний курс осі X, за годинниковою від півночі, °",
    "Tilt sigma": "σ нахилу",
    "Filter 1-sigma of the tilt, per axis, deg": "1σ фільтра для нахилу, на вісь, °",
    "Heading sigma": "σ курсу",
    "Filter 1-sigma of the heading, deg": "1σ фільтра для курсу, °",
    "The filter's estimate of the gyro bias, X Y Z": "Оцінка зсуву гіроскопа фільтром, X Y Z",
    "IMU die temperature, filtered": "Температура кристала IMU, фільтрована",
    "Filter flags": "Прапорці фільтра",
    "No $ATT records: the attitude stream is off (rate 0).": "Записів $ATT немає: потік орієнтації вимкнено (частота 0).",
    "Not connected.": "Не підключено.",
    "Board time %1 s": "Час плати %1 с",
    "paused": "пауза",
    "Pause": "Пауза",
    "Freeze the plots; drag to scroll back in time, wheel to zoom":
        "Зупинити графіки; тягніть, щоб гортати в минуле, коліщатком — масштаб",
    "Window": "Вікно",
    "10 s": "10 с",
    "30 s": "30 с",
    "1 min": "1 хв",
    "2 min": "2 хв",
    "5 min": "5 хв",
    "10 min": "10 хв",
    "Raw data rate": "Частота сирих даних",
    "%1 Hz (raw %2)": "%1 Гц (raw %2)",
    "'stream raw' divisor while a raw plot is shown: 512 Hz divided by this":
        "Дільник 'stream raw', поки показано графік сирих даних: 512 Гц, поділені на нього",
    "Export visible to CSV...": "Експорт видимого в CSV...",
    "Show:": "Показати:",
    "Roll, pitch, yaw": "Крен, тангаж, рискання",
    "deg": "°",
    "Gyro bias estimate": "Оцінка зсуву гіроскопа",
    "deg/s": "°/с",
    "Filter sigma": "σ фільтра",
    "C": "°C",
    "Raw gyro": "Сирий гіроскоп",
    "Raw accelerometer": "Сирий акселерометр",
    "g": "g",
    "Magnetic field magnitude (raw)": "Модуль магнітного поля (сирий)",
    "uT": "мкТл",
    "Barometric altitude": "Барометрична висота",
    "m": "м",
    "Export visible data": "Експорт видимих даних",
    "CSV files (*.csv)": "Файли CSV (*.csv)",
    "Export": "Експорт",
    "Could not write: %1": "Не вдалося записати: %1",
    "Magnetometer ($MAG)": "Магнітометр ($MAG)",
    "Barometer ($BARO)": "Барометр ($BARO)",
    "Write the calibration in force ('cal set' lines) after the header":
        "Записати чинне калібрування (рядки 'cal set') після заголовка",
    "Start recording": "Почати запис",
    "Stop": "Зупинити",
    "Everything the board sends is written to a text file: first the 'info' lines, then the streams, as 'ins10.py log' "
    "writes it. Such a file can be replayed through the filter with the firmware's ins10_replay, or used for calibration "
    "with 'ins10.py ... --from-log' and with the wizards here (Compute from a file).":
        "Усе, що надсилає плата, пишеться в текстовий файл: спершу рядки 'info', далі потоки — так само, як пише "
        "'ins10.py log'. Такий файл можна прогнати через фільтр утилітою прошивки ins10_replay або використати для "
        "калібрування в 'ins10.py ... --from-log' і в майстрах тут («Порахувати з файлу»).",
    "Browse...": "Огляд...",
    "File": "Файл",
    "off": "вимк.",
    "'stream raw n' while recording. 1 is every IMU sample (about 30 kB/s), which ins10_replay needs to run the filter as "
    "the board does.":
        "'stream raw n' під час запису. 1 — кожна вибірка IMU (близько 30 кБ/с); саме це потрібно ins10_replay, щоб "
        "прогнати фільтр так, як на платі.",
    "Raw IMU ($RAW)": "Сирі дані IMU ($RAW)",
    "ins10_replay applies these, so the replay uses the board's calibration; the Python tool ignores them.":
        "ins10_replay їх застосовує, тож відтворення йде з калібруванням плати; інструмент на Python їх ігнорує.",
    "%1 exists. Replace it?": "%1 уже існує. Замінити?",
    "Cannot write %1: %2": "Не вдається записати %1: %2",
    "Stopped: the connection ended.": "Зупинено: з'єднання обірвалося.",
    "Record to": "Записати в",
    "Logs (*.log)": "Журнали (*.log)",
    "Saved %1 lines to %2": "Збережено %1 рядків у %2",
    "Writing %1 failed: %2": "Запис %1 не вдався: %2",
    "Connect a board to record.": "Підключіть плату, щоб записувати.",
    "Recording to %1\n%2 s, %3 lines, %4 kB; lost samples %5, damaged lines %6":
        "Запис у %1\n%2 с, %3 рядків, %4 кБ; втрачено вибірок %5, зіпсованих рядків %6",
    "Settings": "Налаштування",
    # Interface language: Settings and the Language menu.
    "&Language": "&Мова",
    "Language": "Мова",
    "As the system (%1)": "Як у системі (%1)",
    " Hz": " Гц",
    "While connected the application sets this $ATT rate for its views. The board's own rate is put back before every "
    "'cal save' and when disconnecting.":
        "Поки плату підключено, застосунок ставить для своїх сторінок цю частоту $ATT. Власна частота плати "
        "повертається перед кожним 'cal save' і при відключенні.",
    "Attitude rate for viewing": "Частота орієнтації для перегляду",
    "Forget the thermal model...": "Забути температурну модель...",
    "Restart the filter...": "Перезапустити фільтр...",
    "IMU faults": "збої IMU",
    "mag errors": "помилки магнітометра",
    "filter recoveries": "відновлення фільтра",
    "rejected samples": "відкинуті вибірки",
    "accel disturbance": "завада акселерометра",
    "mag deviation": "відхилення поля",
    "Health counters since connecting": "Лічильники справності від підключення",
    "Interference: accelerometer (mg) and magnetometer (uT)": "Завади: акселерометр (мг) і магнітометр (мкТл)",
    "accel, mg": "акселерометр, мг",
    "mag, uT": "магнітометр, мкТл",
    "Key": "Ключ",
    "Value": "Значення",
    "Quantity": "Величина",
    "Refresh": "Оновити",
    "info": "info",
    "status (polled once a second)": "status (опитування раз на секунду)",
    "Learned thermal model of the gyro bias (thermal show)": "Вивчена температурна модель зсуву гіроскопа (thermal show)",
    "c0, dps": "c0, °/с",
    "c1, dps": "c1, °/с",
    "c2, dps": "c2, °/с",
    "bias now, dps": "зсув зараз, °/с",
    "Read again": "Прочитати ще раз",
    "Filter": "Фільтр",
    "Sensors": "Сенсори",
    "Health": "Справність",
    "Flags": "Прапорці",
    "Roll, pitch, yaw, deg": "Крен, тангаж, рискання, °",
    "Heading, deg": "Курс, °",
    "Sigma tilt / heading, deg": "σ нахилу / курсу, °",
    "Gyro bias, dps (sigma)": "Зсув гіроскопа, °/с (σ)",
    "Acceleration, g": "Прискорення, g",
    "Acceleration norm, g": "Норма прискорення, g",
    "Accelerometer disturbance, g": "Завада акселерометра, g",
    "Field, uT": "Поле, мкТл",
    "Field norm / reference, uT": "Норма поля / опорне, мкТл",
    "Dip, deg": "Нахилення, °",
    "Magnetic deviation, uT": "Магнітне відхилення, мкТл",
    "Temperature, C (rate C/s)": "Температура, °C (швидкість °C/с)",
    "Barometer: hPa, C, altitude m": "Барометр: гПа, °C, висота м",
    "IMU samples": "Вибірки IMU",
    "IMU gaps / overflows / bad / bus errors": "IMU: розриви / переповнення / зіпсовані / помилки шини",
    "IMU restarts": "Перезапуски IMU",
    "CPU load, largest batch": "Завантаження процесора, найбільший пакет",
    "%1 %, %2 samples": "%1 %, %2 вибірок",
    "Mag samples / pairs / errors / restarts": "Магнітометр: вибірки / пари / помилки / перезапуски",
    "Baro samples / errors": "Барометр: вибірки / помилки",
    "Filter recoveries / rejected / restarts": "Фільтр: відновлення / відкинуті / перезапуски",
    "No model read yet.": "Модель ще не прочитано.",
    "%1 observations at rest, %2 to %3 C. Bias = c0 + c1 u + c2 u^2, u = (T - 25 C) / 10.":
        "%1 спостережень у спокої, від %2 до %3 °C. Зсув = c0 + c1·u + c2·u², u = (T − 25 °C) / 10.",
    "No observations yet: the board learns the model while it lies still.":
        "Спостережень ще немає: плата вивчає модель, поки лежить нерухомо.",
    "Forget the thermal model": "Забути температурну модель",
    "The board forgets the learned gyro bias-versus-temperature model, and the filter starts again from initial "
    "alignment (leave the board still for 2 s). The stored copy in flash changes only with the next save.\n\nForget it?":
        "Плата забуде вивчену модель «зсув гіроскопа від температури», а фільтр почне знову з початкового вирівнювання "
        "(залиште плату нерухомою на 2 с). Копія у flash зміниться лише з наступним записом.\n\nЗабути?",
    "Thermal model": "Температурна модель",
    "The filter starts again from initial alignment; calibration and the thermal model are kept. Leave the board still "
    "for 2 s.\n\nRestart?":
        "Фільтр почне знову з початкового вирівнювання; калібрування й температурна модель лишаються. Залиште плату "
        "нерухомою на 2 с.\n\nПерезапустити?",
    "roll": "крен",
    "pitch": "тангаж",
    "yaw": "рискання",
    "heading": "курс",
    "bias x": "зсув x",
    "bias y": "зсув y",
    "bias z": "зсув z",
    "tilt sigma": "σ нахилу",
    "heading sigma": "σ курсу",
    "temperature": "температура",
    "gyro x": "гіроскоп x",
    "gyro y": "гіроскоп y",
    "gyro z": "гіроскоп z",
    "accel x": "прискорення x",
    "accel y": "прискорення y",
    "accel z": "прискорення z",
    "mag x": "поле x",
    "mag y": "поле y",
    "mag z": "поле z",
    "|B|": "|B|",
    "altitude": "висота",
    "pressure": "тиск",
    "<b>Temperature: gyro and accelerometer bias drift.</b> 1. Cool the board: unpowered, in a closed bag, 20 minutes in "
    "a fridge. 2. Put it on a solid surface, connect it and do not touch the board or the table. 3. Start recording and "
    "let it warm up for 15-30 minutes; warming it gently with a hair dryer from a distance widens the range. 15 C or "
    "more of range is worth having; less than 3 C is refused.":
        "<b>Температура: дрейф зсуву гіроскопа й акселерометра.</b> 1. Охолодіть плату: без живлення, у закритому "
        "пакеті, 20 хвилин у холодильнику. 2. Покладіть на тверду поверхню, підключіть і не торкайтеся ні плати, ні "
        "столу. 3. Почніть запис і дайте прогрітися 15–30 хвилин; обережний підігрів феном здалеку розширює діапазон. "
        "Варто набрати 15 °C і більше; менше 3 °C не приймається.",
    "IMU": "IMU",
    "Gyro reading against temperature": "Показ гіроскопа залежно від температури",
    "temperature, C": "температура, °C",
    "Temperature now": "Температура зараз",
    "Range needed: at least 3 C, better 15 C or more.": "Потрібен діапазон: щонайменше 3 °C, краще 15 °C і більше.",
    "%1 ... %2 C, span %3": "%1 … %2 °C, діапазон %3",
    "The board moved: those samples will be rejected. Do not touch the board or the table.":
        "Плата рухалася: ці вибірки буде відкинуто. Не торкайтеся плати й столу.",
    "Still.": "Нерухомо.",
    "gyro %1": "гіроскоп %1",
    "Samples, range": "Вибірки, діапазон",
    "%1, %2 ... %3 C": "%1, %2 … %3 °C",
    "Polynomial order, reference": "Порядок полінома, опорна температура",
    "%1, %2 C": "%1, %2 °C",
    "Gyro bias drift over the range": "Дрейф зсуву гіроскопа в діапазоні",
    "%1 deg/s, residual after %2 deg/s": "%1 °/с, залишок після %2 °/с",
    "Gyro bias at the reference, deg/s": "Зсув гіроскопа при опорній температурі, °/с",
    "Gyro slope, deg/s per C": "Температурний коефіцієнт гіроскопа, °/с на °C",
    "Accelerometer drift over the range": "Дрейф акселерометра в діапазоні",
    "%1 mg, residual after %2 mg": "%1 мг, залишок після %2 мг",
    "Accelerometer slope, mg per C": "Температурний коефіцієнт акселерометра, мг на °C",
    "Stop and compute": "Зупинити й порахувати",
    "Compute from a file...": "Порахувати з файлу...",
    "Save the recording...": "Зберегти запис...",
    "A log written by this application or by 'ins10.py log' / --save-log. No board needed.":
        "Журнал, записаний цим застосунком або 'ins10.py log' / --save-log. Плата не потрібна.",
    "Protocol text as 'ins10.py log' writes it: 'info' lines, the calibration in force, then $RAW and $MAG. ins10.py "
    "--from-log and ins10_replay read it.":
        "Текст протоколу так, як пише 'ins10.py log': рядки 'info', чинне калібрування, далі $RAW і $MAG. Його читають "
        "ins10.py --from-log і ins10_replay.",
    "Compute again": "Порахувати ще раз",
    "Cancel computation": "Скасувати розрахунок",
    "Try on the board": "Спробувати на платі",
    "Write the result to the board's RAM. The filter uses it at once (see the Overview); nothing is stored until you save.":
        "Записати результат у RAM плати. Фільтр одразу його використовує (див. «Огляд»); нічого не зберігається, доки "
        "ви не збережете.",
    "Cancel: restore the previous calibration": "Скасувати: повернути попереднє калібрування",
    "Send back the calibration read when the recording started (RAM only).":
        "Надіслати назад калібрування, прочитане на початку запису (лише RAM).",
    "Ready.": "Готово.",
    "Connect a board to record, or compute from a file.": "Підключіть плату для запису або порахуйте з файлу.",
    "Preparing the board...": "Підготовка плати...",
    "Recording: %1 s, %2 IMU rows, %3 magnetometer rows": "Запис: %1 с, рядків IMU %2, рядків магнітометра %3",
    "Recorded %1 s.": "Записано %1 с.",
    "Computing...": "Розрахунок...",
    "Computed from a file.": "Пораховано з файлу.",
    "Computed.": "Пораховано.",
    "Writing to the board...": "Запис на плату...",
    "Not calibrated:": "Не відкалібровано:",
    "Warnings": "Попередження",
    "Fields to write": "Поля для запису",
    "flag bits %1 are added": "додаються біти прапорців %1",
    "Write these fields to the board and save them to flash?\n\n%1\n\nThe flag bits %2 are added to the flag word. The "
    "$ATT rate stored as the power-up rate stays the board's own.":
        "Записати ці поля на плату й зберегти у flash?\n\n%1\n\nДо слова прапорців додаються біти %2. Частота $ATT, що "
        "зберігається як частота після ввімкнення, лишається власною частотою плати.",
    "Recording to compute from": "Запис для розрахунку",
    "Logs (*.log *.txt);;All files (*)": "Журнали (*.log *.txt);;Усі файли (*)",
    "Could not use %1: %2": "Не вдалося використати %1: %2",
    "Save the recording": "Збереження запису",
    "no data yet": "даних ще немає",
    "OK": "Гаразд",
    "Attention": "Увага",
    "Failed": "Збій",
    "Waiting": "Очікування",
    "Attitude initialised": "Орієнтацію ініціалізовано",
    "Aligned while moving; tilt still converging": "Вирівнювання в русі; нахил ще сходиться",
    "Board at rest; zero-rate updates active": "Плата у спокої; діє оновлення нульової швидкості",
    "Heading referenced to the magnetic field": "Курс прив'язано до магнітного поля",
    "Acceleration detected; accelerometer weight reduced at least tenfold":
        "Виявлено прискорення; вагу акселерометра знижено щонайменше вдесятеро",
    "Magnetic disturbance detected; magnetometer weight reduced": "Виявлено магнітну заваду; вагу магнітометра знижено",
    "Magnetometer ignored: not calibrated": "Магнітометр ігнорується: немає калібрування",
    "Sensor saturated or samples lost in the last second": "Насичення сенсора або втрата вибірок за останню секунду",
    "Temperature transient in progress": "Триває температурний перехідний процес",
    "Divergence watchdog acted in the last 5 s": "Сторож розбіжності спрацював за останні 5 с",
    "No usable magnetometer sample for over a second": "Понад секунду немає придатних вибірок магнітометра",
    # Log replay through lib/ins10.
    "Replay a log through the filter": "Відтворити журнал через фільтр",
    "Replay": "Відтворити",
    "Cancel": "Скасувати",
    "Export CSV...": "Експорт CSV...",
    "Runs a recorded log through the firmware's own fusion library (lib/ins10, built from the firmware sources), "
    "exactly as ins10_replay does: what the board's filter would have output on that data. Useful with logs recorded at "
    "'raw 1' and with the magnetometer.":
        "Проганяє записаний журнал через власну бібліотеку фільтра прошивки (lib/ins10, зібрану з сирців прошивки) так "
        "само, як ins10_replay: що видав би фільтр плати на цих даних. Корисно з журналами, записаними з 'raw 1' і "
        "магнітометром.",
    "Log": "Журнал",
    "optional: a saved 'cal show' listing; 'cal set' lines in the log apply too":
        "за бажання: збережений вивід 'cal show'; рядки 'cal set' у журналі теж застосовуються",
    "IMU rate": "Частота IMU",
    "Filter output": "Вихід фільтра",
    "Log to replay": "Журнал для відтворення",
    "Text (*.txt *.cal);;All files (*)": "Текст (*.txt *.cal);;Усі файли (*)",
    "Cannot read %1: %2": "Не вдається прочитати %1: %2",
    "Replaying %1 lines...": "Відтворення %1 рядків...",
    "Cancelled. ": "Скасовано. ",
    "%1 IMU records, %2 s, %3 samples lost in the log; %4 attitude rows.":
        "Записів IMU: %1, %2 с, утрачено в журналі вибірок: %3; рядків орієнтації: %4.",
    "Ignored calibration lines: %1.": "Пропущено рядків калібрування: %1.",
    "Export the replay": "Експорт відтворення",
}


def main() -> int:
    tree = ET.parse(TS)
    root = tree.getroot()
    missing = []
    for message in root.iter("message"):
        source = message.find("source").text or ""
        translation = message.find("translation")
        if source in UK:
            translation.text = UK[source]
            translation.attrib.pop("type", None)
        elif translation.get("type") == "unfinished":
            missing.append(source)
    body = ET.tostring(root, encoding="unicode")
    with open(TS, "w", encoding="utf-8", newline="\n") as handle:
        handle.write('<?xml version="1.0" encoding="utf-8"?>\n<!DOCTYPE TS>\n')
        handle.write(body)
        handle.write("\n")
    for source in missing:
        print(f"untranslated: {source!r}")
    print(f"{len(missing)} untranslated")
    return 1 if missing else 0


if __name__ == "__main__":
    sys.exit(main())
