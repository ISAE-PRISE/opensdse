Per design, you don't need the full hla/certi environment to compile and run the external efcs.
Therefore, we created a new CMakeList.txt file which avoid to search for multiple useless librairies such as Qt5 or CERTI.

To compile the external efcs on real target.

*******************************************
	ON YOUR PC WHERE HLA 
	FEDERATION IS RUNNING
*******************************************
1) Tar the opensdse folder
>> tar -zcvf opensdse.tar.gz opensdse-folder-name.

2) Copy it on the real-time target
>> scp opensdse.tar.gz <USER_NAME>@<TARGET_IP_ADRESS>:</PATH_TO_COPY>

*******************************************
	ON YOUR REAL-TIME TARGET
*******************************************
3) Untar the archive
>> tar -zxvf  opensdse.tar.gz

3-A)(if exists, remove build and run folders)
>> rm -rf build/
>> rm -rf run/

4) Change CMakeLists.txt file

>> cd $OPEN_SDSE_PATH
>> mv $OPEN_SDSE_PATH/src/EXTERNAL_EFCS/CMakeLists_for_ExternalEfcs.txt $OPEN_SDSE_PATH/src/CMakeLists.txt
>> mkdir build
>> mkdir install
>> cd build
>> cmake -DCMAKE_INSTALL_PREFIX=../install ../src 
>> make install
