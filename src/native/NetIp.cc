//          Copyright Boston University SESA Group 2013 - 2014.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
#include "Net.h"

// Is addr a broadcast address on THIS interface?
bool ebbrt::NetworkManager::Interface::ItfAddress::isBroadcast(
    const Ipv4Address& addr) const {
  if (addr.isBroadcast())
    return true;

  if (addr == address)
    return false;

  // if on the same subnet and host bits are all set
  if (isLocalNetwork(addr) &&
      addr.Mask(~netmask) == Ipv4Address::Broadcast().Mask(~netmask))
    return true;

  return false;
}

// Is addr on this subnet?
bool ebbrt::NetworkManager::Interface::ItfAddress::isLocalNetwork(
    const Ipv4Address& addr) const {
  return addr.Mask(netmask) == address.Mask(netmask);
}

// Receive an Ipv4 packet
void ebbrt::NetworkManager::Interface::ReceiveIp(
    EthernetHeader& eth_header, std::unique_ptr<MutIOBuf> buf, uint64_t rxflag) {
  auto packet_len = buf->ComputeChainDataLength();

  //ebbrt::kprintf("%s len->%d\n", __FUNCTION__, packet_len);
  
  if (unlikely(packet_len < sizeof(Ipv4Header)))
    return;

  // ebbrt::kprintf("%s a-1\n", __FUNCTION__);

   auto dp = buf->GetMutDataPointer();
  // auto tbuf = dp.Data();
  // for(int i = 0; i < 20; i++)
  // {
  //   ebbrt::kprintf("0x%02X ", tbuf[i]);
  // }
  // ebbrt::kprintf("\n");
  // ebbrt::kabort("aborting\n");
  
  auto& ip_header = dp.Get<Ipv4Header>();
  //auto& ip_header = dp.Get<Ipv4Header2>();

  /*ebbrt::kprintf("version_ihl -> 0x%02X\n", ip_header.version_ihl);
  ebbrt::kprintf("dscp_ecn -> 0x%02X\n", ip_header.dscp_ecn);
  ebbrt::kprintf("length -> 0x%04X\n", ip_header.length);
  ebbrt::kprintf("proto -> 0x%02X\n", ip_header.proto);
  
  ebbrt::kabort("aborting\n");*/
  
  if (unlikely(ip_header.Version() != 4))
    return;

  //ebbrt::kprintf("%s a\n", __FUNCTION__);
  
  auto hlen = ip_header.HeaderLength();
  if (unlikely(hlen < sizeof(Ipv4Header)))
    return;

  //ebbrt::kprintf("%s b\n", __FUNCTION__);

  auto tot_len = ip_header.TotalLength();
  if (unlikely(packet_len < tot_len))
    return;
  
  //ebbrt::kprintf("%s c\n", __FUNCTION__);

  buf->TrimEnd(packet_len - tot_len);

  if(unlikely((rxflag & RXFLAG_IPCS) == 0)) {
    ebbrt::kprintf("%s RXFLAG_IPCS failed\n");
    return;
  }

  if(unlikely((rxflag & RXFLAG_IPCS_VALID) == 0)) {
    ebbrt::kprintf("%s RXFLAG_IPCS_VALID failed\n");
    return;
  }
  
  //if (unlikely(ip_header.ComputeChecksum() != 0))
  //  return;

  //ebbrt::kprintf("%s d\n", __FUNCTION__);

  auto addr = Address();
  // Unless the protocol is UDP or we have an address on this interface and the
  // packet is directed to us or broadcast on our subnet, then drop. We allow
  // UDP through for DHCP to work before we have an address.
  if (unlikely(ip_header.proto != kIpProtoUDP &&
               (!addr || (!addr->isBroadcast(ip_header.dst) &&
                          addr->address != ip_header.dst))))
    return;

  //ebbrt::kprintf("%s e\n", __FUNCTION__);

  // Drop unacceptable sources
  if (unlikely(ip_header.src.isBroadcast() || ip_header.src.isMulticast()))
    return;

  //ebbrt::kprintf("%s f\n", __FUNCTION__);

  // We do not support fragmentation
  if (unlikely(ip_header.Fragmented()))
    return;

  //ebbrt::kprintf("%s g\n", __FUNCTION__);

  buf->Advance(hlen);

  switch (ip_header.proto) {
  case kIpProtoICMP: {
    //ebbrt::kprintf("%s icmp\n", __FUNCTION__);
    ReceiveIcmp(eth_header, ip_header, std::move(buf));
    break;
  }
  case kIpProtoUDP: {
    //ebbrt::kprintf("%s udp\n", __FUNCTION__);
    ReceiveUdp(ip_header, std::move(buf), rxflag);
    break;
  }
  case kIpProtoTCP: {
    //ebbrt::kprintf("%s tcp\n", __FUNCTION__);
    ReceiveTcp(ip_header, std::move(buf), rxflag);
    break;
  }
  }
}

void ebbrt::NetworkManager::SendIp(std::unique_ptr<MutIOBuf> buf,
                                   Ipv4Address src, Ipv4Address dst,
                                   uint8_t proto, PacketInfo pinfo) {
  // find the interface to route to, then send it on that interface
  auto itf = IpRoute(dst);
  if (likely(itf != nullptr))
    itf->SendIp(std::move(buf), src, dst, proto, std::move(pinfo));
}

// Send IP packet out on this interface
void ebbrt::NetworkManager::Interface::SendIp(std::unique_ptr<MutIOBuf> buf,
                                              Ipv4Address src, Ipv4Address dst,
                                              uint8_t proto, PacketInfo pinfo) {
  buf->Retreat(sizeof(Ipv4Header));
  auto dp = buf->GetMutDataPointer();
  auto& ih = dp.Get<Ipv4Header>();
  ih.version_ihl = 4 << 4 | 5;
  ih.dscp_ecn = 0;
  ih.length = ntohs(buf->ComputeChainDataLength());
  ih.id = 0;
  ih.flags_fragoff = 0;
  ih.ttl = kIpDefaultTtl;
  ih.proto = proto;
  ih.chksum = 0;
  ih.src = src;
  ih.dst = dst;
  ih.chksum = 0;

  // ip checksum offload
  pinfo.flags |= PacketInfo::kNeedsIpCsum;
  
  pinfo.csum_start += sizeof(Ipv4Header);
  pinfo.hdr_len += sizeof(Ipv4Header);

  EthArpSend(kEthTypeIp, ih, std::move(buf), pinfo);
}

// This finds the interface to send on, given a destination
ebbrt::NetworkManager::Interface*
ebbrt::NetworkManager::IpRoute(Ipv4Address dest) {
  // Silly implementation for now, if we have an interface, then return that,
  // otherwise nothing
  if (!interface_)
    return nullptr;

  return &*interface_;
}
