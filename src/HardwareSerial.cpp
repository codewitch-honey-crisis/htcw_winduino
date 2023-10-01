#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <windows.h>

#include "Arduino.h"
#ifndef ARDUINO_SERIAL_EVENT_TASK_STACK_SIZE
#define ARDUINO_SERIAL_EVENT_TASK_STACK_SIZE 2048
#endif

#ifndef ARDUINO_SERIAL_EVENT_TASK_PRIORITY
#define ARDUINO_SERIAL_EVENT_TASK_PRIORITY (configMAX_PRIORITIES - 1)
#endif

#ifndef ARDUINO_SERIAL_EVENT_TASK_RUNNING_CORE
#define ARDUINO_SERIAL_EVENT_TASK_RUNNING_CORE -1
#endif

void serialEvent(void) __attribute__((weak));
void serialEvent(void) {}

#if SOC_UART_NUM > 1

void serialEvent1(void) __attribute__((weak));
void serialEvent1(void) {}
#endif /* SOC_UART_NUM > 1 */

#if SOC_UART_NUM > 2

void serialEvent2(void) __attribute__((weak));
void serialEvent2(void) {}
#endif /* SOC_UART_NUM > 2 */

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_SERIAL)
HardwareSerial Serial(0);
#if SOC_UART_NUM > 1
HardwareSerial Serial1(1);
#endif
#if SOC_UART_NUM > 2
HardwareSerial Serial2(2);
#endif
#if SOC_UART_NUM > 3
HardwareSerial Serial3(3);
#endif

void serialEventRun(void) {
#if ARDUINO_USB_CDC_ON_BOOT  // Serial used for USB CDC
    if (Serial0.available()) serialEvent();
#else
    if (Serial.available()) serialEvent();
#endif
#if SOC_UART_NUM > 1
    if (Serial1.available()) serialEvent1();
#endif
#if SOC_UART_NUM > 2
    if (Serial2.available()) serialEvent2();
#endif
}
#endif

#if !CONFIG_DISABLE_HAL_LOCKS
#define HSERIAL_MUTEX_LOCK() \
    do {                     \
    } while (xSemaphoreTake(_lock, portMAX_DELAY) != pdPASS)
#define HSERIAL_MUTEX_UNLOCK() xSemaphoreGive(_lock)
#else
#define HSERIAL_MUTEX_LOCK()
#define HSERIAL_MUTEX_UNLOCK()
#endif

static DWORD serial_thread_proc(void* state) {
    HardwareSerial& hs = *(HardwareSerial*)state;
    while(true) {
        hs.update();
    }
}

HardwareSerial::HardwareSerial(int uart_nr) : _uart_nr(uart_nr),
                                              _rxBufferSize(256),
                                              _txBufferSize(0),
                                              _onReceiveCB(NULL),
                                              _onReceiveErrorCB(NULL),
                                              _onReceiveTimeout(false),
                                              _rxTimeout(2),
                                              _rxFIFOFull(0),
                                              _rxPin(-1),
                                              _txPin(-1),
                                              _ctsPin(-1),
                                              _rtsPin(-1),
                                              _handle(nullptr),
                                              _quit_event(nullptr),
                                              _has_quit_event(nullptr),
                                              _read_mutex(nullptr),
                                              _thread(nullptr),
                                              _rx_buffer(nullptr),
                                              _rx_size(0),
                                              _rx_cap(0) {
    //_quit_event = CreateEvent(NULL, TRUE, FALSE, NULL);
    //_has_quit_event = CreateEvent(NULL, TRUE, FALSE, NULL);
}
void HardwareSerial::update() {
    if(_handle==nullptr) return;
    uint8_t buf[1024];
    size_t written = 0;
    DWORD dread = sizeof(buf);
    while (dread != 0) {
        if (!ReadFile((HANDLE)_handle, buf, sizeof(buf), &dread, NULL)) {
            break;
        }
        // Serial.printf("Read success\r\n");
        if (dread > 0) {
            if(WAIT_OBJECT_0==WaitForSingleObject((HANDLE)_read_mutex,INFINITE)) {
                size_t ns = _rx_cap;
                while (dread + _rx_size > ns) {
                    ns *= 2;
                }
                if (ns > _rx_cap) {
                    uint8_t *p = (uint8_t *)realloc(_rx_buffer, ns);
                    if (p == nullptr) {
                        dread = 0;
                        ReleaseMutex((HANDLE)_read_mutex);
                        continue;
                    }
                    _rx_buffer = p;
                    _rx_cap = ns;
                }
                memcpy(_rx_buffer + _rx_size, buf, dread);
                _rx_size += dread;
                ReleaseMutex((HANDLE)_read_mutex);
            }
        }
    }
}

HardwareSerial::~HardwareSerial() {
    end();
}

void HardwareSerial::onReceiveError(OnReceiveErrorCb function) {
    // function may be NULL to cancel onReceive() from its respective task
    _onReceiveErrorCB = function;
}

void HardwareSerial::onReceive(OnReceiveCb function, bool onlyOnTimeout) {
    _onReceiveCB = function;

    // setting the callback to NULL will just disable it
    if (_onReceiveCB != NULL) {
        // When Rx timeout is Zero (disabled), there is only one possible option that is callback when FIFO reaches 120 bytes
        _onReceiveTimeout = _rxTimeout > 0 ? onlyOnTimeout : false;
    }
}

// This function allow the user to define how many bytes will trigger an Interrupt that will copy RX FIFO to the internal RX Ringbuffer
// ISR will also move data from FIFO to RX Ringbuffer after a RX Timeout defined in HardwareSerial::setRxTimeout(uint8_t symbols_timeout)
// A low value of FIFO Full bytes will consume more CPU time within the ISR
// A high value of FIFO Full bytes will make the application wait longer to have byte available for the Stkech in a streaming scenario
// Both RX FIFO Full and RX Timeout may affect when onReceive() will be called
void HardwareSerial::setRxFIFOFull(uint8_t fifoBytes) {
    // in case that onReceive() shall work only with RX Timeout, FIFO shall be high
    // this is a work around for an IDF issue with events and low FIFO Full value (< 3)
    if (_onReceiveCB != NULL && _onReceiveTimeout) {
        fifoBytes = 120;
    }
    _rxFIFOFull = fifoBytes;
}

// timout is calculates in time to receive UART symbols at the UART baudrate.
// the estimation is about 11 bits per symbol (SERIAL_8N1)
void HardwareSerial::setRxTimeout(uint8_t symbols_timeout) {
    // Zero disables timeout, thus, onReceive callback will only be called when RX FIFO reaches 120 bytes
    // Any non-zero value will activate onReceive callback based on UART baudrate with about 11 bits per symbol
    _rxTimeout = symbols_timeout;
    if (!symbols_timeout) _onReceiveTimeout = false;  // only when RX timeout is disabled, we also must disable this flag
}

void HardwareSerial::eventQueueReset() {
}

void HardwareSerial::begin(unsigned long baud, uint32_t config, int8_t rxPin, int8_t txPin, bool invert, unsigned long timeout_ms, uint8_t rxfifo_full_thrhd) {
    if (baud == 0) {
        baud = 9600;
    }

    uint16_t comp = 0;
    if (!hardware_get_attached_serial(_uart_nr, &comp)) {
        return;
    }
    if (comp == 0) {
        return;
    }
    bool isopen = _handle != nullptr;
    wchar_t comstr[64];
    wcscpy(comstr, L"\\\\.\\COM");
    _itow(comp, comstr + wcslen(comstr), 10);
    
    if (!isopen) {
        _handle = CreateFileW(comstr,                        // port name
                              GENERIC_READ | GENERIC_WRITE,  // Read/Write
                              0,                             // No Sharing
                              NULL,                          // No Security
                              OPEN_EXISTING,                 // Open existing port only
                              0,                             // Overlapped I/O
                              NULL);                         // Null for Comm Devices
        if (_handle == nullptr || _handle == INVALID_HANDLE_VALUE) {
           return;
        }
    }
    _read_mutex = CreateMutexW(NULL,FALSE,NULL);
    _rx_cap = 1024;
    _rx_buffer = (uint8_t *)malloc(_rx_cap);
    if (_rx_buffer == nullptr) {
        return;
    }
    _rx_size = 0;

 
    DCB dcb;
    dcb.DCBlength = sizeof(DCB);
    GetCommState((HANDLE)_handle, &dcb);
    dcb.fBinary = TRUE;
    dcb.BaudRate = baud;
    dcb.ByteSize = (config & 0xc) >> 2;
    dcb.Parity = (config & 0x3);
    dcb.StopBits = (config & 0x30) >> 4;
    SetCommState((HANDLE)_handle, &dcb);
    _rxPin = rxPin;
    _txPin = txPin;
    _thread=CreateThread(NULL,4000,serial_thread_proc,this,0,NULL);
    // ResetEvent((HANDLE)_quit_event);
    // ResetEvent((HANDLE)_has_quit_event);

}

void HardwareSerial::updateBaudRate(unsigned long baud) {
}

void HardwareSerial::end(bool fullyTerminate) {
    if(_thread!=nullptr) {
        CloseHandle((HANDLE)_thread);
    }
    if (_handle != nullptr) {
    
        CloseHandle((HANDLE)_handle);
        _handle = NULL;
    }
    if(_read_mutex!=nullptr) {
        CloseHandle((HANDLE)_read_mutex);
        _read_mutex = nullptr;
    }
    if (_rx_buffer != nullptr) {
        free(_rx_buffer);
        _rx_buffer = nullptr;
    }
    // default Serial.end() will completely disable HardwareSerial,
    // including any tasks or debug message channel (log_x()) - but not for IDF log messages!
    if (fullyTerminate) {
        _onReceiveCB = NULL;
        _onReceiveErrorCB = NULL;
        _rxFIFOFull = 0;

        _rxPin = _txPin = _ctsPin = _rtsPin = -1;
    }
}

void HardwareSerial::setDebugOutput(bool en) {
}

int HardwareSerial::available(void) {
    if(_read_mutex==nullptr) {
        return 0;
    }
    int result = 0;
    if(WAIT_OBJECT_0==WaitForSingleObject((HANDLE)_read_mutex,INFINITE)) {
        result = _rx_size;
        ReleaseMutex((HANDLE)_read_mutex);
    }
    return result;
}
int HardwareSerial::availableForWrite(void) {
    return _txBufferSize;
}

int HardwareSerial::peek(void) {
    int result = -1;
    if(_read_mutex==nullptr) {
        return result;
    }
    if (WAIT_OBJECT_0 == WaitForSingleObject(_read_mutex, INFINITE)) {
        if (_rx_size > 0) {
            result = *_rx_buffer;
        }
        ReleaseMutex(_read_mutex);
    }
    return result;
}

int HardwareSerial::read(void) {
    int result = -1;
    if(_read_mutex==nullptr) {
        return result;
    }
    if (WAIT_OBJECT_0 == WaitForSingleObject(_read_mutex, INFINITE)) {
        if (_rx_size > 0) {
            result = *_rx_buffer;
            memmove(_rx_buffer, _rx_buffer + 1, --_rx_size);
        }
        ReleaseMutex(_read_mutex);
    }
    return result;
}

// read characters into buffer
// terminates if size characters have been read, or no further are pending
// returns the number of characters placed in the buffer
// the buffer is NOT null terminated.
size_t HardwareSerial::read(uint8_t *buffer, size_t size) {
    size_t result = size;
    if(_read_mutex==nullptr) {
        return 0;
    }
    if (WAIT_OBJECT_0 == WaitForSingleObject(_read_mutex, INFINITE)) {
        if (result >= _rx_size) {
            result = _rx_size;
            memcpy(buffer, _rx_buffer, result);
            _rx_size = 0;
            return result;
        }
        if (_rx_size > 0) {
            memcpy(buffer, _rx_buffer, result);
            size_t ns = _rx_size - result;
            memmove(_rx_buffer, _rx_buffer + result, ns);
            _rx_size = ns;
        }
        ReleaseMutex(_read_mutex);
    }
    return result;
}

// Overrides Stream::readBytes() to be faster using IDF
size_t HardwareSerial::readBytes(uint8_t *buffer, size_t length) {
    return read(buffer, length);
}

void HardwareSerial::flush(void) {
}

void HardwareSerial::flush(bool txOnly) {
}

size_t HardwareSerial::write(uint8_t c) {
    if (_uart_nr == hardware_log_uart) {
        char sz[2];
        sz[1] = 0;
        sz[0] = (char)c;
        log_print(sz);
    }
    return write(&c, 1);
}

size_t HardwareSerial::write(const uint8_t *buffer, size_t size) {
    if (_uart_nr == hardware_log_uart) {
        char *buf = (char *)malloc(size + 1);
        if (buf == NULL) return 0;
        memcpy(buf, buffer, size);
        buf[size] = 0;
        log_print(buf);
        free(buf);
    }
    if (_handle != nullptr) {
        DWORD cb = (DWORD)size;
        WriteFile((HANDLE)_handle,  // Handle to the Serial port
                  buffer,           // Data to be written to the port
                  cb,               // No of bytes to write
                  &cb,              // Bytes written
                  NULL);
        size = (size_t)cb;
    }
    return size;
}
uint32_t HardwareSerial::baudRate()

{
    DCB dcb;
    dcb.DCBlength = sizeof(DCB);
    GetCommState((HANDLE)_handle, &dcb);

    return dcb.BaudRate;
}
HardwareSerial::operator bool() const {
    return true;
}

void HardwareSerial::setRxInvert(bool invert) {
}

// negative Pin value will keep it unmodified
void HardwareSerial::setPins(int8_t rxPin, int8_t txPin, int8_t ctsPin, int8_t rtsPin) {
}

// Enables or disables Hardware Flow Control using RTS and/or CTS pins (must use setAllPins() before)
void HardwareSerial::setHwFlowCtrlMode(uint8_t mode, uint8_t threshold) {
}

size_t HardwareSerial::setRxBufferSize(size_t new_size) {
    _rxBufferSize = new_size;
    return _rxBufferSize;
}

size_t HardwareSerial::setTxBufferSize(size_t new_size) {
    _txBufferSize = new_size;
    return _txBufferSize;
}