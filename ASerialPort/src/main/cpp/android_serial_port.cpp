#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <jni.h>

#include "android/log.h"
#include <jni.h>

static const char *TAG = "android_serial_port";
#define LOGI(fmt, args...) __android_log_print(ANDROID_LOG_INFO,  TAG, fmt, ##args)
#define LOGD(fmt, args...) __android_log_print(ANDROID_LOG_DEBUG, TAG, fmt, ##args)
#define LOGE(fmt, args...) __android_log_print(ANDROID_LOG_ERROR, TAG, fmt, ##args)

static speed_t getBaudrate(jint baudrate) {
    switch (baudrate) {
        case 0:
            return B0;
        case 50:
            return B50;
        case 75:
            return B75;
        case 110:
            return B110;
        case 134:
            return B134;
        case 150:
            return B150;
        case 200:
            return B200;
        case 300:
            return B300;
        case 600:
            return B600;
        case 1200:
            return B1200;
        case 1800:
            return B1800;
        case 2400:
            return B2400;
        case 4800:
            return B4800;
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        case 460800:
            return B460800;
        case 500000:
            return B500000;
        case 576000:
            return B576000;
        case 921600:
            return B921600;
        case 1000000:
            return B1000000;
        case 1152000:
            return B1152000;
        case 1500000:
            return B1500000;
        case 2000000:
            return B2000000;
        case 2500000:
            return B2500000;
        case 3000000:
            return B3000000;
        case 3500000:
            return B3500000;
        case 4000000:
            return B4000000;
        default:
            return -1;
    }
}

static void throwException(JNIEnv *env, const char *name, const char *msg) {
    jclass cls = env->FindClass(name);
    /* if cls is NULL, an exception has already been thrown */
    if (cls != nullptr) {
        env->ThrowNew(cls, msg);
    }

    /* free the local ref */
    env->DeleteLocalRef(cls);
}

extern "C"
JNIEXPORT jobject JNICALL
Java_com_nesp_android_serialport_ASerialPort_open(JNIEnv *env, jclass thiz, jstring path,
                                                  jint baudrate, jint parity, jint data_bits,
                                                  jint stop_bit, jint flags) {
    jboolean iscopy0;
    const char *path_utf0 = env->GetStringUTFChars(path, &iscopy0);
    LOGI("path = %s,baudrate = %d,parity = %d,dataBits = %d,stopBit = %d", path_utf0, baudrate,
         parity, data_bits, stop_bit);
    env->ReleaseStringUTFChars(path, path_utf0);

    int fd;
    speed_t speed;
    jobject mFileDescriptor;

    /* Check arguments */
    {
        speed = getBaudrate(baudrate);
        if (speed == -1) {
            throwException(env, "java/lang/IllegalArgumentException", "Invalid baudrate");
            LOGE("Invalid baudrate");
            return nullptr;
        }

        // if (parity < 0 || parity > 2) {
        if (parity < -1 || parity > 2) {
            throwException(env, "java/lang/IllegalArgumentException", "Invalid parity");
            LOGE("Invalid parity");
            return nullptr;
        }

        // if (data_bits < 5 || data_bits > 8) {
        if (data_bits != -1 && (data_bits < 5 || data_bits > 8)) {
            throwException(env, "java/lang/IllegalArgumentException", "Invalid dataBits");
            LOGE("Invalid dataBits");
            return nullptr;
        }

        // if (stop_bit < 1 || stop_bit > 2) {
        if (stop_bit != -1 && (stop_bit < 1 || stop_bit > 2)) {
            throwException(env, "java/lang/IllegalArgumentException", "Invalid stopBit");
            LOGE("Invalid stopBit");
            return nullptr;
        }
    }

    /* Opening device */
    {
        jboolean iscopy;
        const char *path_utf = env->GetStringUTFChars(path, &iscopy);
        LOGD("Opening serial port %s with flags 0x%x", path_utf, O_RDWR | flags);
        fd = open(path_utf, flags | O_RDWR | O_NOCTTY | O_NDELAY);
        LOGD("open() fd = %d", fd);
        env->ReleaseStringUTFChars(path, path_utf);
        if (fd == -1) {
            /* Throw an exception */
            LOGE("Cannot open port");
            throwException(env, "java/io/IOException", "Cannot open port");
            return nullptr;
        }
    }

    /* Configure device */
    {
        struct termios cfg{};
        LOGD("Configuring serial port");

        if (tcgetattr(fd, &cfg)) {
            LOGE("tcgetattr() failed");
            close(fd);
            throwException(env, "java/io/IOException", "tcgetattr() failed");
            return nullptr;
        }

        cfmakeraw(&cfg);
        cfsetispeed(&cfg, speed);
        cfsetospeed(&cfg, speed);

        // cfg.c_cflag &= ~(ICANON | ECHO | ECHOE | ISIG);

        // cfg.c_cc[VTIME] = 0;
        // cfg.c_cc[VMIN] = 1;

        // cfg.c_cflag &= ~CSIZE;
        // cfg.c_cflag |= (CS8 | CLOCAL | CREAD);
        // cfg.c_iflag = IGNPAR;
        // cfg.c_oflag = 0;
        // cfg.c_lflag = 0;

        switch (parity) {
            case 0:
                cfg.c_cflag &= ~PARENB;
                cfg.c_cflag &= ~INPCK;
                break;
            case 1:
                cfg.c_cflag |= PARENB;
                cfg.c_cflag |= PARODD;
                cfg.c_cflag |= INPCK;
                break;
            case 2:
                cfg.c_cflag |= PARENB;
                cfg.c_cflag &= ~PARODD;
                cfg.c_cflag |= INPCK;
                break;
            default:
                break;
        }

        // cfg.c_cflag &= ~CSIZE;

        switch (data_bits) {
            case 5:
                cfg.c_cflag |= CS5;
                break;
            case 6:
                cfg.c_cflag |= CS6;
                break;
            case 7:
                cfg.c_cflag |= CS7;
                break;
            case 8:
                cfg.c_cflag |= CS8;
                break;
            default:
                break;
        }

        switch (stop_bit) {
            case 1:
                cfg.c_cflag &= ~CSTOPB;
                break;
            case 2:
                cfg.c_cflag |= CSTOPB;
                break;
            default:
                break;
        }

        if (tcsetattr(fd, TCSANOW, &cfg)) {
            LOGE("tcsetattr() failed");
            close(fd);
            /* TODO: throw an exception */
            return nullptr;
        }
    }

    /* Create a corresponding file descriptor */
    {
        jclass cFileDescriptor = env->FindClass("java/io/FileDescriptor");
        jmethodID iFileDescriptor = env->GetMethodID(cFileDescriptor, "<init>", "()V");
        jfieldID descriptorID = env->GetFieldID(cFileDescriptor, "descriptor", "I");
        mFileDescriptor = env->NewObject(cFileDescriptor, iFileDescriptor);
        env->SetIntField(mFileDescriptor, descriptorID, (jint) fd);
    }

    return mFileDescriptor;
}

extern "C"
JNIEXPORT void JNICALL
Java_com_nesp_android_serialport_ASerialPort_close(JNIEnv *env, jobject thiz) {
    jclass SerialPortClass = env->GetObjectClass(thiz);
    jclass FileDescriptorClass = env->FindClass("java/io/FileDescriptor");

    jfieldID mFdID = env->GetFieldID(SerialPortClass, "mFd", "Ljava/io/FileDescriptor;");
    jfieldID descriptorID = env->GetFieldID(FileDescriptorClass, "descriptor", "I");

    jobject mFd = env->GetObjectField(thiz, mFdID);
    jint descriptor = env->GetIntField(mFd, descriptorID);

    LOGD("close(fd = %d)", descriptor);
    close(descriptor);
}