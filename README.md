# README #

Прощивка для FPGA SPARTAN-6 семейства для частотомера РЧ-24 производства СКТБ ЭлПА

### Для чего этот репозиторий? ###

* Репозиторий содержит полный набот файлов для сборки и отладки прошивки многоканального частотмера РЧ-24
* Версия: 0.8

### Как собирать? ###

##### Подготовка к сборке

* Необходимые пакеты: cmake, git, python2, python3, Xilinx ISE 14.3, protobuf
* Опциональные пакеты: doxygen

##### Сборка компилятора #####

Процессор [AltoOR32](http://opencores.org/project,altor32) требует специфический компилятор.

1. Создаём отдельный каталог для работы и переходим в него

1. Скачать [готовый тулчеин OR32](https://github.com/openrisc/newlib/releases/download/v2.3.0-1/or1k-elf_gcc5.2.0_binutils2.26_newlib2.3.0-1_gdb7.11.tgz). Распаковать куда-либо, но не добавлять в PATH. Он нам не совсем подходит, однако поможет в сборке

1. Скачиваем исходники
	```{r, engine='bash', clone_gdb}
	git clone https://github.com/openrisc/binutils-gdb.git --depth 1 --branch gdb-7.11-or1k
	```
	```{r, engine='bash', clone_or1k_gcc}
	git clone https://github.com/openrisc/or1k-gcc.git --depth 1 --branch or1k-5.2.0
	```
	```{r, engine='bash', clone_newlib}
	git clone https://github.com/openrisc/newlib.git --depth 1 --branch or1k
	```

1. Скачиваем и распаковываем необхдимые для сборки gcc библиотеки, устанавливаем симлинки.
	```{r, engine='bash', get_gmp}	
	wget https://gmplib.org/download/gmp/gmp-6.1.0.tar.xz
	tar -xf gmp-6.1.0.tar.xz	
	ln -s gmp-6.1.0 or1k-gcc/gmp
	```
	```{r, engine='bash', get_mpc}
	wget ftp://ftp.gnu.org/gnu/mpc/mpc-1.0.3.tar.gz
	tar -xzf mpc-1.0.3.tar.gz
	ln -s mpc-1.0.3 or1k-gcc/mpc
	```
	```{r, engine='bash', get_mpfr}
	wget http://www.mpfr.org/mpfr-current/mpfr-3.1.4.tar.xz
	tar -xf mpfr-3.1.4.tar.xz
	ln -s mpfr-3.1.4 or1k-gcc/mpfr
	```

1. Устанавливаем Переменные окружения
	```{r, engine='bash', ENV}
	export TARGET=or1knd-elf
	export PREFIX=/opt/or1knd-elf
	```

1. Собираем binutils
	```{r, engine='bash', build_binutils}
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

1. Собираем первую стадию gcc. Необходимо отключить использование аппаратного умножения.
	```{r, engine='bash', build_gcc1}
	cd ../or1k-gcc
	echo "MULTILIB_EXTRA_OPTS = msoft-mul msoft-div" >> gcc/config/or1k/t-or1knd
	mkdir build1 && cd build1
	../configure --target=$TARGET --prefix=$PREFIX \
		--enable-languages=c --disable-shared --disable-libssp \
		--disable-werror --disable-multilib
	make
	sudo make install
	```

1. Собираем newlib.		
Необходимо добавить в **PATH** первую стадию gcc, также следует сделать симлинк на  **or1knd-elf-ranlib** иначе **make install** не может найти **ranlib** и не сработает.
	```{r, engine='bash', build_nlib}
	sudo ln -s $PREFIX/bin/or1knd-elf-ranlib /usr/bin
	export PATH=$PATH:$PREFIX/bin
	cd ../newlib
	mkdir build && cd build
	../configure --target=$TARGET --prefix=$PREFIX
	make
	sudo make install
	sudo rm /usr/bin/or1knd-elf-ranlib # удаляем симлинк
	```

1. Собираем вторую стадию gcc.
	```{r, engine='bash', build_gcc2}
	cd ../or1k-gcc
	mkdir build2 && cd build2
	../configure --target=$TARGET --prefix=$PREFIX \
		--enable-languages=c --disable-shared --disable-libssp \
		--with-newlib
	make
	sudo make install
	```

1. Проверяем результат сборки
	```{r, engine='bash', test_build}
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

1. GDB можно не собирать а воспользоваться готовым, например сделав симлинк. Можно и собрать свой. Для Debian-подобных систем понадобится пакет **python-dev**
	```{r, engine='bash', build_gdb}
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

1. Добавть каталог **/opt/or1knd-elf** в **PATH**, чтобы можно было обращаться к компилятору из системы сборки

### Конфигурация ###
* Клонируйте репозиторий в удобное вам место, перейдите в него и инициализируйте сабмодули
	```{r, engine='bash', clone_repo}
	git clone https://Sctb_Elpa@bitbucket.org/Sctb_Elpa/or1k-spartan6-freqmeter.git
	cd or1k-spartan6-freqmeter
	git submodule --init update
	```

* Создайте каталог для продуктов сборки и прейдите в него, затем запустите генерацию cmake
	```{r, engine='bash', cmake_config}
	mkdir build && cd build
	cmake ..
	```

* Настройка проекта. В случае возникновения проблем с конфигурацией cmake список настроек будет отображаться неполностью, пока вы не устраните соответствующую проблему.
	Запустите cmake-gui
	```{r, engine='bash', cmake_config}
	cmake-gui .
	```
	В появившемся окне включите флажки *Grouped* и *Advanced*
	- - - - -
	Назначение настраиваемых параметров
	* Ungrouped entries/OR1KND - исполняемый файл компилятора. Убедитесь, что он определен верно
	* BAUD/BAUD_I2C - частота шины i2c в герцах
	* BAUD/BAUD_MDIO - частота шины mdio в герцах
	* BAUD/BAUD_SPI_CLK_DEVIDER_LEN - длина делителя частоты шины SPI		
 (Fspi = Fcpu / (2 ^ BAUD_SPI_CLK_DEVIDER_LEN))
	* BAUD/BAUD_UART0 - скорость отладочного интерфейса UART0
	* CLOCK/CLOCK_CPU_CLOCK_DEVIDER - делитель частоты CPU
	* CLOCK/CLOCK_CPU_CLOCK_MULTIPLYER - Множитель частоты CPU		

	Fcpu = Fin * CLOCK_CPU_CLOCK_MULTIPLYER / CLOCK_CPU_CLOCK_DEVIDER		
	Стабильность работы проверена на частоте процессора 66 МГц

	* CLOCK/CLOCK_FREF_CLOCK_DEVIDER - делитель опорной частоты 
	* CLOCK/CLOCK_FREF_CLOCK_MULTIPLYER - Множитель опорной частоты 		

	Fref = Fin * CLOCK_REF_CLOCK_MULTIPLYER / CLOCK_REF_CLOCK_DEVIDER		
	Частотомер проверен на частоте 100 МГц
	
	* DEVICE/DEVICE_BOARD_NAME - название целевой платы (список доступных в каталоге hdl/ucf)
	* DEVICE/DEVICE_CHIP_NAME - название чипа FPGA
	* DEVICE/REF_CLOCK_HZ - частота опорного генератора в герцах
	* DEVICE/DEVICE_SPI_FLASH_CHIP - название загрузочной микросхемы flash-памяти (список доступных в программе impact)
	* ETHERNET/ETHERNET_MAC_ADDRESS_FOCRСE - Использовать указанный MAC аддресс (для отладки)
		* True - использовать этот адресс
		* False - Генерировать новый MAC-адресс.
	* ETHERNET/ETHERNET_MAC_ADDRESS_FOCRСED - MAC-адрес, присваиваемый устройству в принудительном режиме
	* ETHERNET/ETHERNET_MAC_ADDRESS_MSB - старший байт MAC-адреса в режиме генерации
	* ETHERNET/ETHERNET_SKIP_UDP_CHECKSUMS - пропустрить проверку и генерацию контрольной суммы UDP-пакета (повышает быстродействие системы)
	* ETHERNET/ETHERNET_STATIC_IP_ADDR - IP-адрес для статического режима работы (без DHCP)
	* ETHERNET/ETHERNET_STATIC_IP_GATEWAY - IP-адрес шлюза по-умолчанию для статического режима работы (без DHCP)
	* ETHERNET/ETHERNET_STATIC_IP_NETMASK - маска подсети для статического режима работы (без DHCP)
	* ETHERNET/ETHERNET_USE_DHCP - Активирует режим работы автоматическим получением настроек сети от сервера DHCP
	* PERIPHERIAL/PERIPHERIAL_ENABLE_CRC32 - Включить модуль аппаратного вычисления контрольных сумм по алгоритму CRC32
	* PERIPHERIAL/PERIPHERIAL_ENABLE_ETHERNET - Включить модуль ethernet
	* PERIPHERIAL/PERIPHERIAL_ENABLE_GPIO - Включить модуль GPIO
	* PERIPHERIAL/PERIPHERIAL_ENABLE_HW_MUL - Включить модуль аппаратного умножения
	* PERIPHERIAL/PERIPHERIAL_ENABLE_I2C - Включить модуль аппаратного i2c
	* PERIPHERIAL/PERIPHERIAL_ENABLE_TIMER - Включить модуль таймеров
	* PERIPHERIAL/PERIPHERIAL_ENABLE_UART0 - Включить модуль UART0 (отладочный)
	* SERVER/SERVER_HTTP - Включить HTTP сервер (порт 80)
	* SERVER/SERVER_UDP - Включить сервер UDP
	* SERVER/SERVER_UDP_PORT - Порт UDP-сервера
	* SERVER/SERVER_WEBSOC - Включить сервер websocket
	* SERVER/SERVER_WEBSOC_PORT - Порт сервера websocket
	* SIM/SIM_TEST_CRC32 - Добавить в сборку тест модуля CRC32
	* SIM/SIM_TEST_FREQMETER - Добавить в сборку тест модуля частотомера
	* SIM/SIM_TEST_GPIO - Добавить в сборку тест модуля GPIO
	* SIM/SIM_TEST_I2C - Добавить в сборку тест модуля i2c
	* SIM/SIM_TEST_MDIO - Добавить в сборку тест модуля MDIO
	* SIM/SIM_TEST_MINMAC - Добавить в сборку общий тест модуля Ethernet
	* SIM/SIM_TEST_MINMAC_SLOT_LOGICK - Добавить в сборку тест логики приёмных слотов модуля Ethernet
	* SIM/SIM_TEST_MULTIPLICATION - Добавить в сборку тест умножения
	* SITE/SITE_FILEEXT_MAX - максимальная длина расширения файла для веб-сервера
	* SITE/SITE_FILENAME_MAX - максимальная длина имени файла для веб-сервера
	* SITE/SITE_STATIC_CACHE - кэш данных для веб-сервера (в Байтах)
	* SYSTEM/SYSTEM_FPGA_BMEMORY_USE - число блоков блочной памяти FPGA, используемых как системная память
	* SYSTEM/SYSTEM_FREF_COUNTER_HYBRID - использовать гибридный счетчик для опроной частоты. (двоичный + счетчик кода грея)
	* SYSTEM/SYSTEM_FREF_COUNTER_LEN - длина счетчика опроной частоты в битах
	* SYSTEM/SYSTEM_FREQ_TYPE - тип данных для хранения и расчета частоты. Доступны варианты: float или double
	* SYSTEM/SYSTEM_HEAP_SIZE - размер системной кучи в байтах
	* SYSTEM/SYSTEM_INPUTS_COUNTER_LEN - длина счетчиков измеряемой частоты в битах
	* SYSTEM/SYSTEM_INPUTS_F_IN_MAX - максимальная теоритическая измеряемая частота в герцах
	* SYSTEM/SYSTEM_MEASURE_TIME_DEFAULT - время измерения по-умолчанию в милисекундах
	* SYSTEM/SYSTEM_MEASURE_TIME_MIN - минимально-допустимое время изменеия в милисекундах
	* SYSTEM/SYSTEM_TRAP_EARLY - Ранний старт отладчика (до загрузки приложения)
	* XILINX/XILINX_DIR - Каталог в котором находятся исполняемые файлы Xilinx ISE

	**ВОЗДЕРЖИТЕСЬ ОТ ПРАВКИ ОСТАЛЬНЫХ ПАРАМЕТРОВ, ЕСЛИ НЕ ЗНАИЛЕ ЗА ЧТО ОНИ ОТВЕЧАЮТ**

### Запуск тестов ###
* Тесты поведения
	1. Тест модулей FPGA находятся в каталоге hdl/testbench. Для их запуска выполните команду

		```{r, engine='bash', cmake_config}
		make tb.<название файла теста>.run
		```

		После компиляции откроется окно Xilinx iSIM, где можно запустить тест и просмотреть результаты. *Важно:* Некторые тесты требуют определенной программы, включите соответствующий флажок в разделе SIM конфигурации cmake, для перед началом сборки теста. Не используйте более одного флажка одновременно, иначе программы будет содержать код всех выбранных тестов и в окне симуляции возникнут проблемы с интерапритацией результатов.

	1. Тесты интеграции устройства и ПК.
		* Соберите, и зашейте программу в FPGA, присоедините устройство к сети Etherenet так, чтобы тестовый ПК мог отправлять данные устройству.
		* Убедитесь, что связь успешно установлена при помощи команды **ping**
		* Запустите сценарий тестировния

			```{r, engine='bash', cmake_config}
			EST_IP=<IP-адрес устройства> make pb_pytest.run 
			```
			
### Прошивка ###
Прошивка устройства производится при помощи програматора Xilinx Platform Cable USB


### Contribution guidelines ###

* Writing tests
* Code review
* Other guidelines

### Who do I talk to? ###

* Repo owner or admin
* Other community or team contact
