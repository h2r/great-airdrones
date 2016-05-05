#!/bin/bash

curl http://www.infradead.org/\~tgr/libnl/files/libnl-1.1.4.tar.gz | tar xvz;
curl http://w1.fi/releases/wpa_supplicant-0.7.3.tar.gz | tar xvz;
curl http://www.openssl.org/source/openssl-1.0.1e.tar.gz | tar xvz;

mkdir -p sources;
mv ./wpa_supplicant-0.7.3 sources/wpa_supplicant;
mv ./libnl-1.1.4 sources/libnl;
mv ./openssl-1.0.1e sources/openssl;


homedir=$(pwd)
builddir=${homedir}/build
installdir=${homedir}/install

export CCFLAGS="-march=armv7-a"

cd ./sources/libnl

./configure --prefix=${builddir}/usr \
            --host=arm-none-linux-gnueabi \
            --build=i686-pc-linux-gnu || exit 1;

sed -i 's/-g //g'    Makefile.opts;
sed -i 's/-O2/-Os/g' Makefile.opts;

make || exit 1;
make install || exit 1;

cd ${homedir}

cd ./sources/openssl

export ARCH=arm
export CROSS_COMPILE=arm-none-linux-gnueabi-

./Configure --prefix=${builddir}/usr linux-armv4 || exit 1;

sed -i 's/-O3/-Os/g' Makefile;

make || exit 1;
make install_sw || exit 1;

unset ARCH
unset CROSS_COMPILE

cd ${homedir}

cd sources/wpa_supplicant/wpa_supplicant

cp defconfig .config;
mv Makefile Makefile.backup;

sed -i 's/-g$//g'    Makefile.backup;
sed -i 's/-O2/-Os/g' Makefile.backup;
sed -i 's/-lcrypto/-lcrypto -ldl/g' Makefile.backup;
sed "16 i \\\nDESTDIR = ${builddir}\nCFLAGS += -I${builddir}/usr/include\nLDFLAGS += -L${builddir}/usr/lib\n" Makefile.backup > Makefile;

rm Makefile.backup;

make CC=arm-none-linux-gnueabi-gcc || exit 1;
make install || exit 1;

cd ${homedir}

unset CCFLAGS

mkdir -p ./install;
cp ${builddir}/usr/local/sbin/wpa* ${installdir}/;
