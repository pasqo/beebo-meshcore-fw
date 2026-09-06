#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <vector>
#include <string>

// Mock Stream class for native testing. Base class kept minimal/back-compat
// (Utils.h only ever needed print()) -- every other method defaults to a
// harmless "empty/not connected" value so code that includes this but never
// calls the extra methods keeps compiling unaffected.
class Stream {
public:
    virtual void print(char c) {}
    virtual void print(const char* str) {}
    virtual int available() { return 0; }
    virtual int read() { return -1; }
    virtual int peek() { return -1; }
    virtual size_t write(uint8_t b) { return 0; }
    virtual size_t write(const uint8_t* buf, size_t len) { return 0; }
    virtual int readBytes(uint8_t* buf, size_t len) { return 0; }
    virtual int availableForWrite() { return 0; }
};

// beebo: test double for DualModeSerialInterface's native unit tests -- a
// plain byte queue for RX (feed() pushes bytes a test wants "arriving on the
// wire"; available()/read() drain it) and a capture buffer for TX (every
// write() call appends here, tx() lets a test inspect what was sent). No
// short-write/partial-write simulation -- writeAll()'s own retry logic isn't
// what these tests are exercising.
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
    int readBytes(uint8_t* buf, size_t len) override {
        size_t n = 0;
        while (n < len && _rx_pos < _rx.size()) buf[n++] = _rx[_rx_pos++];
        return (int)n;
    }

    size_t write(uint8_t b) override { _tx.push_back(b); return 1; }
    size_t write(const uint8_t* buf, size_t len) override {
        for (size_t i = 0; i < len; i++) _tx.push_back(buf[i]);
        return len;
    }
    void print(char c) override { _tx.push_back((uint8_t)c); }
    void print(const char* str) override { for (const char* p = str; *p; p++) _tx.push_back((uint8_t)*p); }
    int availableForWrite() override { return 64; }

    const std::vector<uint8_t>& tx() const { return _tx; }
    void clearTx() { _tx.clear(); }

private:
    std::vector<uint8_t> _rx;
    size_t _rx_pos = 0;
    std::vector<uint8_t> _tx;
};
