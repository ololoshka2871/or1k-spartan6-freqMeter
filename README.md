# README #

Прощивка для FPGA SPARTAN-6 семейства для частотомера РЧ-24 производства СКТБ ЭлПА

### Для чего этот репозиторий? ###

* Репозиторий содержит полный набор файлов для сборки и отладки прошивки многоканального частотмера РЧ-24
* Версия: 0.8

### Как собирать? ###

#### Подготовка к сборке

* Необходимые пакеты: cmake, git, python2, python3, Xilinx ISE 14.3, protobuf
* Опциональные пакеты: doxygen

#### Сборка компилятора

Процессор [AltoOR32](http://opencores.org/project,altor32) требует специфический компилятор.

* Создаём отдельный каталог для работы и переходим в него

* Скачать [готовый тулчеин OR32](https://github.com/openrisc/newlib/releases/download/v2.3.0-1/or1k-elf_gcc5.2.0_binutils2.26_newlib2.3.0-1_gdb7.1*tgz). Распаковать куда-либо, но не добавлять в PATH. Он нам не совсем подходит, однако поможет в сборке

* Скачиваем исходники		
```
#!bash
git clone https://github.com/openrisc/binutils-gdb.git --depth 1 --branch gdb-7.11-or1k
```
```
#!bash
git clone https://github.com/openrisc/or1k-gcc.git --depth 1 --branch or1k-5.2.0
```
```
#!bash
git clone https://github.com/openrisc/newlib.git --depth 1 --branch or1k
```

* Скачиваем и распаковываем необхдимые для сборки gcc библиотеки, устанавливаем симлинки.		
```
#!bash	
wget https://gmplib.org/download/gmp/gmp-6.*0.tar.xz
tar -xf gmp-6.*0.tar.xz	
ln -s gmp-6.*0 or1k-gcc/gmp
```
```
#!bash
wget ftp://ftp.gnu.org/gnu/mpc/mpc-*0.3.tar.gz
tar -xzf mpc-*0.3.tar.gz
ln -s mpc-*0.3 or1k-gcc/mpc
```
```
#!bash
wget http://www.mpfr.org/mpfr-current/mpfr-3.*4.tar.xz
tar -xf mpfr-3.*4.tar.xz
ln -s mpfr-3.*4 or1k-gcc/mpfr
```

* Устанавливаем Переменные окружения		
```
#!bash
export TARGET=or1knd-elf
export PREFIX=/opt/or1knd-elf
```

* Собираем binutils		
```
#!bash
cd binutils-gdb
mkdir build && cd build
../configure --target=$TARGET --prefix=$PREFIX \
	--enable-shared --disable-itcl --disable-tk \
	--disable-tcl --disable-winsup --disable-libgui \
	--disable-rda --disable-sid --disable-sim --disable-gdb \
	--with-sysroot --disable-newlib --disable-libgloss \
	--disable-werror
make
sudo make install
```
*Примечание: Если вылетит ошибка связанная с yywrap воспользоваться инструкцией [тут](https://stackoverflow.com/questions/24925247/undefined-reference-to-yywrap)*

* Собираем первую стадию gcc. Необходимо отключить использование аппаратного умножения.		
```
#!bash
cd ../or1k-gcc
echo "MULTILIB_EXTRA_OPTS = msoft-mul msoft-div" >> gcc/config/or1k/t-or1knd
mkdir build1 && cd build1
../configure --target=$TARGET --prefix=$PREFIX \
	--enable-languages=c --disable-shared --disable-libssp \
	--disable-werror --disable-multilib
make
sudo make install
```

* Собираем newlib.		
Необходимо добавить в **PATH** первую стадию gcc, также следует сделать симлинк на  **or1knd-elf-ranlib** иначе **make install** не может найти **ranlib** и не сработает.		
```
#!bash
sudo ln -s $PREFIX/bin/or1knd-elf-ranlib /usr/bin
export PATH=$PATH:$PREFIX/bin
cd ../newlib
mkdir build && cd build
../configure --target=$TARGET --prefix=$PREFIX
make
sudo make install
sudo rm /usr/bin/or1knd-elf-ranlib # удаляем симлинк
```

* Собираем вторую стадию gcc.		
```
#!bash
cd ../or1k-gcc
mkdir build2 && cd build2
../configure --target=$TARGET --prefix=$PREFIX \
	--enable-languages=c --disable-shared --disable-libssp \
	--with-newlib
make
sudo make install
```

* Проверяем результат сборки		
```
#!bash
$PREFIX/bin/or1knd-elf-gcc -print-multi-lib
# Примерный результат вывода
.;@msoft-mul@msoft-div
delay;@mdelay@msoft-mul@msoft-div
compat-delay;@mcompat-delay@msoft-mul@msoft-div
soft-float;@msoft-float@msoft-mul@msoft-div
delay/soft-float;@mdelay@msoft-float@msoft-mul@msoft-div
compat-delay/soft-float;@mcompat-delay@msoft-float@msoft-mul@msoft-div
```
*Как видно, все варианты библиотек gcc собраны с @msoft-mul@msoft-div*

* GDB можно не собирать а воспользоваться готовым, например сделав симлинк. Можно и собрать свой. Для Debian-подобных систем понадобится пакет **python-dev**		
```
#!bash
cd binutils-gdb
mkdir build-gdb && cd build-gdb
../configure --target=$TARGET --prefix=$PREFIX \
	--enable-shared --disable-itcl --disable-tk \
	--disable-tcl --disable-winsup --disable-libgui \
	--disable-rda --disable-sid --disable-sim \
	--with-sysroot --disable-newlib --disable-libgloss \
	--with-python=yes
make
sudo make install
```

* Добавть каталог **/opt/or1knd-elf** в **PATH**, чтобы можно было обращаться к компилятору из системы сборки		

#### Конфигурация
* Клонируйте репозиторий в удобное вам место, перейдите в него и инициализируйте сабмодули		
```
#!bash
git clone https://Sctb_Elpa@bitbucket.org/Sctb_Elpa/or1k-spartan6-freqmeter.git
cd or1k-spartan6-freqmeter
git submodule --init update
```

* Создайте каталог для продуктов сборки и прейдите в него, затем запустите генерацию cmake		
```
#!bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
```

* Настройка проекта. В случае возникновения проблем с конфигурацией cmake список настроек будет отображаться неполностью, пока вы не устраните соответствующую проблему.		
	Запустите cmake-gui
```
#!bash
cmake-gui .
```

* В появившемся окне включите флажки *Grouped* и *Advanced*
	Назначение настраиваемых параметров
	* OR1KND - исполняемый файл компилятора. Убедитесь, что он определен верно
	* BAUD_I2C - частота шины i2c в герцах
	* BAUD_MDIO - частота шины mdio в герцах
	* BAUD_SPI_CLK_DEVIDER_LEN - длина делителя частоты шины SPI		
 (Fspi = Fcpu / (2 ^ BAUD_SPI_CLK_DEVIDER_LEN))
	* BAUD_UART0 - скорость отладочного интерфейса UART0
        * CLOCK_USE_PLL - Использовать умножение частоты, или напрямую подавать опорную частоту на блоки системы
	* CLOCK_CPU_CLOCK_DEVIDER - делитель частоты CPU
	* CLOCK_CPU_CLOCK_MULTIPLYER - Множитель частоты CPU		

	Fcpu = Fin * CLOCK_CPU_CLOCK_MULTIPLYER / CLOCK_CPU_CLOCK_DEVIDER		
	Стабильность работы проверена на частоте процессора 66 МГц

	* CLOCK_FREF_CLOCK_DEVIDER - делитель опорной частоты 
	* CLOCK_FREF_CLOCK_MULTIPLYER - Множитель опорной частоты 		

	Fref = Fin * CLOCK_REF_CLOCK_MULTIPLYER / CLOCK_REF_CLOCK_DEVIDER		
	Частотомер проверен на частоте 100 МГц
	
	* DEVICE_BOARD_NAME - название целевой платы (список доступных в каталоге hdl/ucf)
	* DEVICE_CHIP_NAME - название чипа FPGA
	* REF_CLOCK_HZ - частота опорного генератора в герцах
	* DEVICE_SPI_FLASH_CHIP - название загрузочной микросхемы flash-памяти (список доступных в программе impact)
	* ETHERNET_MAC_ADDRESS_FOCRСE - Использовать указанный MAC аддресс (для отладки)
	* ETHERNET_MAC_ADDRESS_FOCRСED - MAC-адрес, присваиваемый устройству в принудительном режиме
	* ETHERNET_MAC_ADDRESS_MSB - старший байт MAC-адреса в режиме генерации
	* ETHERNET_SKIP_UDP_CHECKSUMS - пропустрить проверку и генерацию контрольной суммы UDP-пакета (повышает быстродействие системы)
	* ETHERNET_STATIC_IP_ADDR - IP-адрес для статического режима работы (без DHCP)
	* ETHERNET_STATIC_IP_GATEWAY - IP-адрес шлюза по-умолчанию для статического режима работы (без DHCP)
	* ETHERNET_STATIC_IP_NETMASK - маска подсети для статического режима работы (без DHCP)
	* ETHERNET_USE_DHCP - Активирует режим работы автоматическим получением настроек сети от сервера DHCP
	* PERIPHERIAL_ENABLE_CRC32 - Включить модуль аппаратного вычисления контрольных сумм по алгоритму CRC32
	* PERIPHERIAL_ENABLE_ETHERNET - Включить модуль ethernet
	* PERIPHERIAL_ENABLE_GPIO - Включить модуль GPIO
	* PERIPHERIAL_ENABLE_HW_MUL - Включить модуль аппаратного умножения
	* PERIPHERIAL_ENABLE_I2C - Включить модуль аппаратного i2c
	* PERIPHERIAL_ENABLE_TIMER - Включить модуль таймеров
	* PERIPHERIAL_ENABLE_UART0 - Включить модуль UART0 (отладочный)
	* SERVER_HTTP - Включить HTTP сервер (порт 80)
	* SERVER_UDP - Включить сервер UDP
	* SERVER_UDP_PORT - Порт UDP-сервера
	* SERVER_WEBSOC - Включить сервер websocket
	* SERVER_WEBSOC_PORT - Порт сервера websocket
	* SIM_TEST_CRC32 - Добавить в сборку тест модуля CRC32
	* SIM_TEST_FREQMETER - Добавить в сборку тест модуля частотомера
	* SIM_TEST_GPIO - Добавить в сборку тест модуля GPIO
	* SIM_TEST_I2C - Добавить в сборку тест модуля i2c
	* SIM_TEST_MDIO - Добавить в сборку тест модуля MDIO
	* SIM_TEST_MINMAC - Добавить в сборку общий тест модуля Ethernet
	* SIM_TEST_MINMAC_SLOT_LOGICK - Добавить в сборку тест логики приёмных слотов модуля Ethernet
	* SIM_TEST_MULTIPLICATION - Добавить в сборку тест умножения
	* SITE_FILEEXT_MAX - максимальная длина расширения файла для веб-сервера
	* SITE_FILENAME_MAX - максимальная длина имени файла для веб-сервера
	* SITE_STATIC_CACHE - кэш данных для веб-сервера (в Байтах)
	* SYSTEM_FPGA_BMEMORY_USE - число блоков блочной памяти FPGA, используемых как системная память
	* SYSTEM_FREF_COUNTER_HYBRID - использовать гибридный счетчик для опроной частоты. (двоичный + счетчик кода грея)
	* SYSTEM_FREF_COUNTER_LEN - длина счетчика опроной частоты в битах
	* SYSTEM_FREQ_TYPE - тип данных для хранения и расчета частоты. Доступны варианты: float или double
	* SYSTEM_HEAP_SIZE - размер системной кучи в байтах
	* SYSTEM_INPUTS_COUNTER_LEN - длина счетчиков измеряемой частоты в битах
	* SYSTEM_INPUTS_F_IN_MAX - максимальная теоритическая измеряемая частота в герцах
	* SYSTEM_MEASURE_TIME_DEFAULT - время измерения по-умолчанию в милисекундах
	* SYSTEM_MEASURE_TIME_MIN - минимально-допустимое время изменеия в милисекундах
	* SYSTEM_TRAP_EARLY - Ранний старт отладчика (до загрузки приложения)
	* XILINX_DIR - Каталог в котором находятся исполняемые файлы Xilinx ISE

**ВОЗДЕРЖИТЕСЬ ОТ ПРАВКИ ОСТАЛЬНЫХ ПАРАМЕТРОВ, ЕСЛИ НЕ ЗНАЕТЕ ЗА ЧТО ОНИ ОТВЕЧАЮТ**

#### Запуск тестов FPGA

* Тесты поведения		
	* Тест модулей FPGA находятся в каталоге hdl/testbench. Для их запуска выполните команду
	* После компиляции откроется окно Xilinx iSIM, где можно запустить тест и просмотреть результаты. *Важно:* Некторые тесты требуют определенной программы, включите соответствующий флажок в разделе SIM конфигурации cmake, для перед началом сборки теста. Не используйте более одного флажка одновременно, иначе программы будет содержать код всех выбранных тестов и в окне симуляции возникнут проблемы с интерапритацией результатов.		

```
#!bash
make tb.<название файла теста>.run
```

* Тесты интеграции устройства и ПК.		
	* Соберите, и зашейте программу в FPGA, присоедините устройство к сети Etherenet так, чтобы тестовый ПК мог отправлять данные устройству.
	* Убедитесь, что связь успешно установлена при помощи команды **ping**
	* Запустите сценарий тестировния		

```
#!bash
TEST_IP=<IP-адрес устройства> make pb_pytest.run 
```
			
#### Прошивка

Прошивка устройства производится при помощи програматора Xilinx Platform Cable USB. 
* Установка программатора		
	Необходим пакет fxload.
	Создайте правило udev (**/etc/udev/rules.d/xusbdfwu.rules**) со следующим содержимым:		

```
#!bash
ATTRS{idVendor}=="03fd", ATTRS{idProduct}=="0008", MODE="666"
SUBSYSTEMS=="usb", ACTION=="add", ATTRS{idVendor}=="03fd", ATTRS{idProduct}=="0007", RUN+="/sbin/fxload -v -t fx2 -I /path/to/xusbdfwu.hex -D $tempnode"
SUBSYSTEMS=="usb", ACTION=="add", ATTRS{idVendor}=="03fd", ATTRS{idProduct}=="0009", RUN+="/sbin/fxload -v -t fx2 -I /path/to/xusb_xup.hex -D $tempnode"
SUBSYSTEMS=="usb", ACTION=="add", ATTRS{idVendor}=="03fd", ATTRS{idProduct}=="000d", RUN+="/sbin/fxload -v -t fx2 -I /path/to/xusb_emb.hex -D $tempnode"
SUBSYSTEMS=="usb", ACTION=="add", ATTRS{idVendor}=="03fd", ATTRS{idProduct}=="000f", RUN+="/sbin/fxload -v -t fx2 -I /path/to/xusb_xlp.hex -D $tempnode"
SUBSYSTEMS=="usb", ACTION=="add", ATTRS{idVendor}=="03fd", ATTRS{idProduct}=="0013", RUN+="/sbin/fxload -v -t fx2 -I /path/to/xusb_xp2.hex -D $tempnode"
SUBSYSTEMS=="usb", ACTION=="add", ATTRS{idVendor}=="03fd", ATTRS{idProduct}=="0015", RUN+="/sbin/fxload -v -t fx2 -I /path/to/xusb_xse.hex -D $tempnode"
```

Здесь **/path/to** Это подкаталог в каталоге установки Xilinx ISE, **14.3/ISE_DS/common/bin/lin{32/64}**
* Перезагрузите правила udev:		

```
#!bash
sudo udevadm control --reload-rules
```

* Отключите и вновь подключите программатор
* Прошивка		
	Выполните команду при подключенном к плате программаторе

```
#!bash
make flash
```

* Возможна ситуация, когда прошивка закончится неудачей из-за неверных привилегий у пользователя. Используйте привилегии администатора **sudo** в этом случае. Однако это может повредить систему сборки, если произойдет генерация новых файлов с привилегиями администратора, чтобы этого избежать используйте **sudo** только на этапе непосредственно прошивки.

### Структура проекта ###

```
├── application		# Основное приложение
│   ├── ibexukIP		# Сетевой стек 
│   ├── libbase		# Общие модули
│   ├── libhw			# Драйверы устройств
│   ├── nanopb		# Библиотека доступа по протокуолу protobuf
│   ├── scripts		# Скрипты автоматизации процесса сборки
│   └── servers		# Сетевые серверы
├── bootloader		# Загрузчик
│   ├── gdb_stub		# Драйвер отладочного моста UART
│   └── testsrc		# Тестовые программы для поддержки тестирования аппаратных модулей
├── cmake_modules	# вспомогательные модули cmake
├── doc				# Документация
├── hdl				# Исходные коды конфигурации FPGA
│   ├── altor32		# Ядро процессора AltOr32
│   ├── iicmb			# Модуль i2c
│   ├── mdio			# Модуль MDIO
│   ├── memory		# Модули реализующие области памяти
│   ├── myminimac		# модуль Ethernet
│   ├── scripts		# Скрипты автоматизации процесса сборки 
│   ├── soc			# Вспомогательные модульи поддержки для "системы на чипе"
│   ├── testbench		# Тесты аппаратных модулей
│   ├── ucf			# Файлы конфигурации конкретной плат
│   └── utils			# Общие вспомогательные моули
├── tests			# Тесты, запускаемые со стороны ПК
├── tools			# Вспомогательные средства сборки
└── www				# Файлы внутреннего веб-сервера
    ├── scripts		# Скрипты автоматизации процесса сборки
    └── siteroot		# Корневой каталог сайта
        ├── api		# Внешний API
        ├── css		# Стили
        ├── fonts	# Шрифты
        ├── img		# Изображения
        ├── js		# Клиентские скрипты
        └── pages	# HTML страницы
```

### Вспомогательные возможности ###

#### Требования для отладки

* Отладка произвдится при помощи моста UART. Необходимо иметь разрешенный аппаратный модуль в настройках проекта.
* Необходимо собрать проект в режиме отладки чтобы выходной файл прошивки содержал отладочную информацию, кроме того в этом режиме активирован драйвер отладки gdb_stub. Для этого создайте отдельное дерево сборки в каталоге проекта и сконфигурируйте cmake		
```
#!bash
mkdir debug-build && cd debug-build
cmake .. -DCMAKE_BUILD_TYPE=Debug
```
* Подключите выводы RX, TX и GND на отлаживаемом устройстве к соответствующим контактам переходника USB-UART. **ВНИМАНИЕ!!!** *Используйте только переходники с логическими уровнями 3,3 В, иначе возможно повреждение FPFA*

#### Проверка работы драйвера отладки

* Выполните команду, чтобы открыть последовательный порт, Где **/dev/ttyUSB0** - Ваш переходник USB-UART

```
#!bash
screen /dev/ttyUSB0 115200
```

* В соседнем терминале выполните команду прошивки:

```
#!bash
make flash
```
* По окончании которой вы должны увидеть в терминале screen сообщение вида: 		
```
#!bash
A
B
C
D
$T0521:10001e48;01:1000dfb4;#4b
```

* Подключение отладчика: выполните команду, открывающую подключение к последовательному порту
		
```
#!bash
tools/mkgdb_server.sh
```

* В соседнем терминале выполните запуск отладчика
		
```
#!bash
or1knd-elf-gdb application/app.elf
```

* Затем в консоли отладчика gdb

```
(gdb) set remote interrupt-on-connect
(gdb) target remote :3333
```
		
* В случае успешного подключения отладчик готов к работе. Теперь можно настройить интеграцию отладки с IDE.		
*ПРИМЕЧАНИЕ 1: отладчик довольно нестабилен, поэтому не используйте отладку по шагам, лучше установить точку останова в требуемом месте и дождаться её срабатывания, проанализировать локальные переменные и перезапустить программу с начала*
*ПРИМЕЧАНИЕ 2: в отладочном режиме программа не стартует автоматически а останавливается сразу после загрузки приложения перед входом в функцию main(), и ожидает подключания отладчика*

#### Тестирование

* Тестирование чтения.		
	Протестировать работоспособность каналов можно используя python-скрипт pb_reader.py, запуск которого производится следующим образом		

```
#!bash
TEST_IP=<IP-адрес устройства> make pb_reader.run
```
		
* Граф вызовов функций С		
	Можно построить и вывезти специальный граф вызовов функций приложения и сохранить его в SVG. Требуется **Doxygen**:		

```
#!bash
make application_call_graph
```
* Генерация нового MAC-адреса		
	Каждое новое устройство должно иметь уникальный (сгенерированный случайным образом) MAC-адрес. Перед прошивкой нового экземпляда устройства выполните генерацию нового MAC-адреса:
*Примечание: При перешивании уже исползовавшегося устройства новый MAC адресс не будет принят. Необходимо сбросить настройки извлечением батареи часов.*			

```
#!bash
make regen_mac
```