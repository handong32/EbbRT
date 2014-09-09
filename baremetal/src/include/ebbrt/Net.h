//          Copyright Boston University SESA Group 2013 - 2014.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
#ifndef BAREMETAL_SRC_INCLUDE_EBBRT_NET_H_
#define BAREMETAL_SRC_INCLUDE_EBBRT_NET_H_

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#include <boost/container/list.hpp>
#pragma GCC diagnostic pop

#include <ebbrt/AtomicUniquePtr.h>
#include <ebbrt/Clock.h>
#include <ebbrt/EventManager.h>
#include <ebbrt/IOBuf.h>
#include <ebbrt/NetDhcp.h>
#include <ebbrt/NetEth.h>
#include <ebbrt/NetIp.h>
#include <ebbrt/NetTcp.h>
#include <ebbrt/SharedPoolAllocator.h>
#include <ebbrt/RcuTable.h>
#include <ebbrt/StaticSharedEbb.h>

namespace ebbrt {
class EthernetDevice {
 public:
  virtual void Send(std::unique_ptr<IOBuf> buf) = 0;
  virtual const EthernetAddress& GetMacAddress() = 0;
  virtual void Poll() = 0;
  virtual ~EthernetDevice() {}
};

class NetworkManager : public StaticSharedEbb<NetworkManager> {
 public:
  struct UdpEntry {
    RcuHListHook hook;
    uint16_t port{0};
    MovableFunction<void(Ipv4Address, uint16_t, std::unique_ptr<MutIOBuf>)>
    func;
  };

  class Interface;

  class UdpPcb {
   public:
    UdpPcb() : entry_{new UdpEntry()} {}
    uint16_t Bind(uint16_t port);
    void Receive(MovableFunction<
        void(Ipv4Address, uint16_t, std::unique_ptr<MutIOBuf>)> func);
    void SendTo(Ipv4Address addr, uint16_t port, std::unique_ptr<IOBuf> buf);
    Future<void> Close();

   private:
    struct UdpEntryDeleter {
      void operator()(UdpEntry* e);
    };
    std::unique_ptr<UdpEntry, UdpEntryDeleter> entry_;

    friend class Interface;
  };

  struct TcpSegment {
    TcpSegment(std::unique_ptr<MutIOBuf> buf, TcpHeader& th, uint16_t tcp_len)
        : buf(std::move(buf)), th(th), tcp_len(tcp_len) {}

    size_t SeqLen() { return tcp_len; }

    std::unique_ptr<MutIOBuf> buf;
    TcpHeader& th;
    uint16_t tcp_len;
  };

  class TcpPcb;

  struct ListeningTcpEntry : public CacheAligned {
    void Input(const Ipv4Header& ih, const TcpHeader& th, const TcpInfo& info,
               std::unique_ptr<MutIOBuf> buf);
    RcuHListHook hook;
    uint16_t port{0};
    MovableFunction<void(TcpPcb)> accept_fn;
  };

  class ListeningTcpPcb {
   public:
    ListeningTcpPcb() : entry_{new ListeningTcpEntry()} {}
    uint16_t Bind(uint16_t port, MovableFunction<void(TcpPcb)> accept);

   private:
    struct ListeningTcpEntryDeleter {
      void operator()(ListeningTcpEntry* e);
    };
    std::unique_ptr<ListeningTcpEntry, ListeningTcpEntryDeleter> entry_;
    friend class Interface;
    friend class ListeningTcpEntry;
  };

  class ITcpHandler {
   public:
    virtual void Receive(std::unique_ptr<MutIOBuf> buf) = 0;
    virtual void WindowIncrease(uint16_t new_window) = 0;
    virtual ~ITcpHandler() {}
  };

  struct TcpEntry : public CacheAligned, public Timer::Hook {
    TcpEntry() = default;
    TcpEntry(MovableFunction<void(TcpPcb)>*);
    void Fire() override;
    void EnqueueSegment(TcpHeader& th, std::unique_ptr<MutIOBuf> buf,
                        uint16_t flags);
    void Input(const Ipv4Header& ih, const TcpHeader& th, const TcpInfo& info,
               std::unique_ptr<MutIOBuf> buf);
    void Receive(const Ipv4Header& ih, const TcpHeader& th, const TcpInfo& info,
                 std::unique_ptr<MutIOBuf> buf);
    size_t Output(ebbrt::clock::Wall::time_point now);
    uint16_t WindowRemaining();
    void SetTimer(ebbrt::clock::Wall::time_point now);
    void SendSegment(TcpSegment& segment);
    void SendEmptyAck();
    void Close();
    void SendFin();
    void Send(std::unique_ptr<IOBuf> buf);

    RcuHListHook hook;
    Ipv4Address address;
    std::tuple<Ipv4Address, uint16_t, uint16_t> key;
    boost::container::list<TcpSegment> unacked_segments;
    boost::container::list<TcpSegment> pending_segments;
    enum State {
      kSynReceived,
      kEstablished,
      kCloseWait,
      kLastAck,
      kClosed
    } state;
    uint32_t snd_lbb;  // seqno of next byte to send
    uint32_t rcv_nxt;  // The next byte we expect to be received
    uint32_t last_sent_ack;  // The last ackno we sent
    uint32_t lastack;  // next byte to be acked by receiver
    uint16_t rcv_wnd;  // last window the remote receiver advertised
    ebbrt::clock::Wall::time_point retransmit;  // when to retransmit
    MovableFunction<void(TcpPcb)>* accept_fn;
    std::unique_ptr<ITcpHandler> handler;
    bool window_notify;
    bool timer_set;
  };

  class TcpPcb {
   public:
    TcpPcb() = default;
    explicit TcpPcb(TcpEntry* entry) : entry_(entry) {}
    void InstallHandler(std::unique_ptr<ITcpHandler> handler);
    uint16_t RemoteWindowRemaining();
    void SetWindowNotify(bool notify);
    void Send(std::unique_ptr<IOBuf> buf);

   private:
    struct TcpEntryDeleter {
      void operator()(TcpEntry* e);
    };
    std::unique_ptr<TcpEntry, TcpEntryDeleter> entry_;

    friend class Interface;
    friend class TcpEntry;
  };

  class Interface {
   public:
    struct ItfAddress {
      bool isBroadcast(const Ipv4Address& addr) const;
      bool isLocalNetwork(const Ipv4Address& addr) const;

      Ipv4Address address;
      Ipv4Address netmask;
      Ipv4Address gateway;
    };

    struct ItfAddressDeleter {
      void operator()(ItfAddress* p) {
        event_manager->DoRcu([p]() { delete p; });
      }
    };

    explicit Interface(EthernetDevice& ether_dev)
        : address_(nullptr), ether_dev_(ether_dev) {}

    void Receive(std::unique_ptr<MutIOBuf> buf);
    void Send(std::unique_ptr<IOBuf> buf);
    void SendUdp(UdpPcb& pcb, Ipv4Address addr, uint16_t port,
                 std::unique_ptr<IOBuf> buf);
    void SendIp(std::unique_ptr<MutIOBuf> buf, Ipv4Address src, Ipv4Address dst,
                uint8_t proto);
    void Poll();
    const EthernetAddress& MacAddress();
    const ItfAddress* Address() const { return address_.get(); }
    void SetAddress(std::unique_ptr<ItfAddress> address) {
      address_.store(address.release());
    }
    Future<void> StartDhcp();

   private:
    struct DhcpPcb : public CacheAligned, public Timer::Hook {
      void Fire() override;

      UdpPcb udp_pcb;
      enum State { kInactive, kSelecting, kRequesting, kBound } state;
      uint32_t xid;
      size_t tries;
      size_t options_len;
      ebbrt::clock::Wall::time_point lease_time;
      ebbrt::clock::Wall::time_point renewal_time;
      ebbrt::clock::Wall::time_point rebind_time;
      Promise<void> complete;
    };

    void ReceiveArp(EthernetHeader& eh, std::unique_ptr<MutIOBuf> buf);
    void ReceiveIp(EthernetHeader& eh, std::unique_ptr<MutIOBuf> buf);
    void ReceiveIcmp(EthernetHeader& eh, Ipv4Header& ih,
                     std::unique_ptr<MutIOBuf> buf);
    void ReceiveUdp(Ipv4Header& ih, std::unique_ptr<MutIOBuf> buf);
    void ReceiveTcp(const Ipv4Header& ih, std::unique_ptr<MutIOBuf> buf);
    void ReceiveDhcp(Ipv4Address from_addr, uint16_t from_port,
                     std::unique_ptr<MutIOBuf> buf);
    void EthArpSend(uint16_t proto, const Ipv4Header& ih,
                    std::unique_ptr<MutIOBuf> buf);
    void EthArpRequest(ArpEntry& entry);
    void DhcpOption(DhcpMessage& message, uint8_t option_type,
                    uint8_t option_len);
    void DhcpOptionByte(DhcpMessage& message, uint8_t value);
    void DhcpOptionShort(DhcpMessage& message, uint16_t value);
    void DhcpOptionLong(DhcpMessage& message, uint32_t value);
    boost::optional<uint8_t> DhcpGetOptionByte(const DhcpMessage& message,
                                               uint8_t option_type);
    boost::optional<uint16_t> DhcpGetOptionShort(const DhcpMessage& message,
                                                 uint8_t option_type);
    boost::optional<uint32_t> DhcpGetOptionLong(const DhcpMessage& message,
                                                uint8_t option_type);
    void DhcpOptionTrailer(DhcpMessage& message);
    void DhcpSetState(DhcpPcb::State state);
    void DhcpCreateMessage(DhcpMessage& msg);
    void DhcpDiscover();
    void DhcpHandleOffer(const DhcpMessage& message);
    void DhcpHandleAck(const DhcpMessage& message);

    atomic_unique_ptr<ItfAddress, ItfAddressDeleter> address_;
    EthernetDevice& ether_dev_;
    DhcpPcb dhcp_pcb_;
  };

  static void Init();

  Interface& NewInterface(EthernetDevice& ether_dev);

 private:
  Future<void> StartDhcp();
  void SendIp(std::unique_ptr<MutIOBuf> buf, Ipv4Address src, Ipv4Address dst,
              uint8_t proto);
  void TcpReset(uint32_t seqno, uint32_t ackno, const Ipv4Address& local_ip,
                const Ipv4Address& remote_ip, uint16_t local_port,
                uint16_t remote_port);
  Interface* IpRoute(Ipv4Address dest);

  std::unique_ptr<Interface> interface_;
  RcuHashTable<ArpEntry, Ipv4Address, &ArpEntry::hook, &ArpEntry::paddr>
  arp_cache_{8};  // 256 buckets
  RcuHashTable<UdpEntry, uint16_t, &UdpEntry::hook, &UdpEntry::port> udp_pcbs_{
      8};  // 256 buckets
  RcuHashTable<ListeningTcpEntry, uint16_t, &ListeningTcpEntry::hook,
               &ListeningTcpEntry::port> listening_tcp_pcbs_{8};  // 256 buckets
  RcuHashTable<TcpEntry, std::tuple<Ipv4Address, uint16_t, uint16_t>,
               &TcpEntry::hook, &TcpEntry::key,
               boost::hash<std::tuple<Ipv4Address, uint16_t, uint16_t>>>
  tcp_pcbs_{8};  // 256 buckets
  EbbRef<SharedPoolAllocator<uint16_t>> udp_port_allocator_{
      SharedPoolAllocator<uint16_t>::Create(49152, 65535,
                                            ebb_allocator->AllocateLocal())};
  EbbRef<SharedPoolAllocator<uint16_t>> tcp_port_allocator_{
      SharedPoolAllocator<uint16_t>::Create(49152, 65535,
                                            ebb_allocator->AllocateLocal())};

  alignas(cache_size) std::mutex arp_write_lock_;
  alignas(cache_size) std::mutex udp_write_lock_;
  alignas(cache_size) std::mutex listening_tcp_write_lock_;
  alignas(cache_size) std::mutex tcp_write_lock_;

  friend void ebbrt::Main(ebbrt::multiboot::Information* mbi);
};

constexpr auto network_manager = EbbRef<NetworkManager>(kNetworkManagerId);
}  // namespace ebbrt

#endif  // BAREMETAL_SRC_INCLUDE_EBBRT_NET_H_
