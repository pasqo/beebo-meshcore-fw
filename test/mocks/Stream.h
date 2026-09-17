#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <vector>

// Mock Stream class for native testing. Provides minimal interface needed by
// Utils.h, matching real Arduino's own Print -> Stream hierarchy.

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

class Print
{
public:
    virtual size_t write(uint8_t b) { return 1; }
    size_t write(const char *str)
    {
        if(str == NULL) {
            return 0;
        }
        return write((const uint8_t *) str, strlen(str));
    }
    virtual size_t write(const uint8_t *buffer, size_t size) {
        size_t t = 0;
        for (int i = 0; i < size; i++) { t += write(buffer[i]); }
        return t;
    }
    size_t write(const char *buffer, size_t size)
    {
        return write((const uint8_t *) buffer, size);
    }

    virtual size_t print(unsigned char b, int r = DEC) { return 0; }
    virtual size_t print(int v, int r = DEC) { return 0; }
    virtual size_t print(unsigned int v, int r = DEC) { return 0; }
    virtual size_t print(long v, int r = DEC) { return 0; }
    virtual size_t print(unsigned long v, int r = DEC) { return 0; }
    virtual size_t print(long long v, int r = DEC) { return 0; }
    virtual size_t print(unsigned long long v, int r = DEC) { return 0; }
    virtual size_t print(double v, int p = 2) { return 0; }

    size_t print(char c) { return write(c); }
    size_t print(const char* str) { return write(str); }

    //size_t println(void)  { return 0; }

    virtual void flush() { /* Empty implementation for backward compatibility */ }
};

class Stream: public Print
{
public:
    virtual ~Stream() = default;
    virtual int available() { return 0; }
    virtual int availableForWrite() { return 0; }
    virtual int read() { return -1; }
    virtual int peek() { return 0; }

    virtual size_t readBytes(char *buffer, size_t length) {
        size_t i = 0;
        while (i < length && available()) {
            buffer[i++] = read();
        }
        return i;
    }
    virtual size_t readBytes(uint8_t *buffer, size_t length)
    {
        return readBytes((char *) buffer, length);
    }
};

// beebo: test double for DualModeSerialInterface's native unit tests -- a
// plain byte queue for RX (feed() pushes bytes a test wants "arriving on the
// wire"; available()/read() drain it) and a capture buffer for TX (every
// write() call appends here, tx() lets a test inspect what was sent). No
// short-write/partial-write simulation -- writeAll()'s own retry logic isn't
// what these tests are exercising. print()/write(const char*) come from
// Print's own non-virtual implementations, which route through the
// overridden write(uint8_t)/write(const uint8_t*, size_t) below, so no
// separate print() overrides are needed here.
class FakeStream : public Stream {
public:
    void feed(uint8_t b) { _rx.push_back(b); }
    void feed(const uint8_t* buf, size_t len) { for (size_t i = 0; i < len; i++) _rx.push_back(buf[i]); }

    int available() override { return (int)(_rx.size() - _rx_pos); }
    int read() override {
        if (_rx_pos >= _rx.size()) return -1;
        return _rx[_rx_pos++];
    }
    int peek() override {
        if (_rx_pos >= _rx.size()) return -1;
        return _rx[_rx_pos];
    }
    size_t readBytes(uint8_t* buf, size_t len) override {
        size_t n = 0;
        while (n < len && _rx_pos < _rx.size()) buf[n++] = _rx[_rx_pos++];
        return n;
    }

    size_t write(uint8_t b) override { _tx.push_back(b); return 1; }
    size_t write(const uint8_t* buf, size_t len) override {
        for (size_t i = 0; i < len; i++) _tx.push_back(buf[i]);
        return len;
    }
    int availableForWrite() override { return 64; }

    const std::vector<uint8_t>& tx() const { return _tx; }
    void clearTx() { _tx.clear(); }

private:
    std::vector<uint8_t> _rx;
    size_t _rx_pos = 0;
    std::vector<uint8_t> _tx;
};
