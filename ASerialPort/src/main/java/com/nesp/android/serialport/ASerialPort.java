package com.nesp.android.serialport;

import android.util.Log;

import java.io.File;
import java.io.FileDescriptor;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

public class ASerialPort {
    private static final String TAG = "ASerialPort";

    private FileDescriptor mFd;
    private FileInputStream mFileInputStream;
    private FileOutputStream mFileOutputStream;

    public ASerialPort(final String suFilePath,
                       final String port,
                       final int baudrate,
                       int flags) throws SecurityException, IOException {
        this(suFilePath, port, baudrate, -1, -1, -1, flags);
    }

    public ASerialPort(final String suFilePath,
                       final String port,
                       final int baudrate,
                       final int parity,
                       int flags) throws SecurityException, IOException {
        this(suFilePath, port, baudrate, parity, -1, -1, flags);
    }

    public ASerialPort(final String suFilePath,
                       final String port,
                       final int baudrate,
                       final int parity,
                       final int dataBits,
                       final int stopBit,
                       int flags
    ) throws SecurityException, IOException {
        File portDevice = new File(port);
        /* Check access permission */
        if (!portDevice.canRead() || !portDevice.canWrite()) {
            try {
                /* Missing read/write permission, trying to chmod the file */
                Process su;
                su = Runtime.getRuntime().exec((suFilePath == null || suFilePath.isEmpty()) ? "/system/bin/su" : suFilePath);
                String cmd = "chmod 666 " + portDevice.getAbsolutePath() + "\n"
                        + "exit\n";
                su.getOutputStream().write(cmd.getBytes());
                if ((su.waitFor() != 0) || !portDevice.canRead()
                        || !portDevice.canWrite()) {
                    throw new SecurityException();
                }
            } catch (Exception e) {
                e.printStackTrace();
                throw new SecurityException();
            }
        }

        mFd = open(portDevice.getAbsolutePath(), baudrate, parity, dataBits, stopBit, flags);
        if (mFd == null) {
            Log.e(TAG, "native open returns null");
            throw new IOException();
        }
        mFileInputStream = new FileInputStream(mFd);
        mFileOutputStream = new FileOutputStream(mFd);
    }

    // Getters and setters
    public InputStream getInputStream() {
        return mFileInputStream;
    }

    public OutputStream getOutputStream() {
        return mFileOutputStream;
    }

    // JNI
    private native static FileDescriptor open(String path, int baudrate, int parity, int dataBits, int stopBit, int flags);

    public native void close();

    static {
        System.loadLibrary("android_serial_port");
    }

}
