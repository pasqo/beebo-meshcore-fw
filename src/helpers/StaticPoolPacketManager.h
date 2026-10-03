#pragma once

#include <Dispatcher.h>

class PacketQueue {
  mesh::Packet** _table;
  uint8_t* _pri_table;
  uint32_t* _schedule_table;
  int _size, _num;

public:
  PacketQueue(int max_entries);
  mesh::Packet* get(uint32_t now);
  bool add(mesh::Packet* packet, uint8_t priority, uint32_t scheduled_for);
  int count() const { return _num; }
  int countBefore(uint32_t now) const;
  mesh::Packet* itemAt(int i) const { return _table[i]; }
  mesh::Packet* removeByIdx(int i);
};

class StaticPoolPacketManager : public mesh::PacketManager {
  PacketQueue unused, send_queue, rx_queue;
  uint32_t _tx_queue_full_count = 0;
  uint32_t _rx_queue_full_count = 0;

public:
  StaticPoolPacketManager(int pool_size);

  mesh::Packet* allocNew() override;
  void free(mesh::Packet* packet) override;
  bool queueOutbound(mesh::Packet* packet, uint8_t priority, uint32_t scheduled_for) override;
  mesh::Packet* getNextOutbound(uint32_t now) override;
  int getOutboundCount(uint32_t now) const override;
  int getOutboundTotal() const override;
  int getFreeCount() const override;
  mesh::Packet* getOutboundByIdx(int i) override;
  mesh::Packet* removeOutboundByIdx(int i) override;
  bool queueInbound(mesh::Packet* packet, uint32_t scheduled_for) override;
  mesh::Packet* getNextInbound(uint32_t now) override;
  uint32_t getTxQueueFullCount() const override { return _tx_queue_full_count; }
  uint32_t getRxQueueFullCount() const override { return _rx_queue_full_count; }
};