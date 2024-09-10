.. SPDX-License-Identifier: GPL-2.0

=================
Linux Kernel QUIC
=================

Introduction
============

The QUIC protocol, as defined in RFC9000, offers a UDP-based, secure
transport with flow-controlled streams for efficient communication,
low-latency connection setup, and network path migration, ensuring
confidentiality, integrity, and availability across various deployments.

This implementation introduces QUIC support in Linux Kernel, offering
several key advantages:

- Seamless Integration for Kernel Subsystems: Kernel subsystems such as
  SMB and NFS can operate over QUIC seamlessly after the handshake,
  leveraging the net/handshake APIs.

- Standardized Socket APIs for QUIC: This implementation standardizes the
  socket APIs for QUIC, covering essential operations like listen, accept,
  connect, sendmsg, recvmsg, close, get/setsockopt, and getsock/peername().

- Efficient ALPN Routing: It incorporates ALPN routing within the kernel,
  efficiently directing incoming requests to the appropriate applications
  across different processes based on ALPN.

- Performance Enhancements: By minimizing data duplication through
  zero-copy techniques such as sendfile(), and paving the way for crypto
  offloading in NICs, this implementation enhances performance and prepares
  for future optimizations.

This implementation offers fundamental support for the following RFCs:

- RFC9000 - QUIC: A UDP-Based Multiplexed and Secure Transport
- RFC9001 - Using TLS to Secure QUIC
- RFC9002 - QUIC Loss Detection and Congestion Control
- RFC9221 - An Unreliable Datagram Extension to QUIC
- RFC9287 - Greasing the QUIC Bit
- RFC9368 - Compatible Version Negotiation for QUIC
- RFC9369 - QUIC Version 2

The socket APIs for QUIC follow the RFC draft [1]:

- The Sockets API Extensions for In-kernel QUIC Implementations

Implementation
==============

The core idea is to implement QUIC within the kernel, using a userspace
handshake approach.

Only the processing and creation of raw TLS Handshake Messages are handled
in userspace, facilitated by a TLS library like GnuTLS. These messages are
exchanged between kernel and userspace via sendmsg() and recvmsg(), with
cryptographic details conveyed through control messages (cmsg).

The entire QUIC protocol, aside from the TLS Handshake Messages processing
and creation, is managed within the kernel. Rather than using a Upper Layer
Protocol (ULP) layer, this implementation establishes a socket of type
IPPROTO_QUIC (similar to IPPROTO_MPTCP), operating over UDP tunnels.

Kernel consumers can initiate a handshake request from the kernel to
userspace using the existing net/handshake netlink. The userspace
component, such as tlshd service [2], then manages the processing
of the QUIC handshake request.

- Handshake Architecture:

  ┌──────┐  ┌──────┐
  │ APP1 │  │ APP2 │ ...
  └──────┘  └──────┘
  ┌──────────────────────────────────────────┐
  │     {quic_client/server_handshake()}     │<─────────────┐
  └──────────────────────────────────────────┘       ┌─────────────┐
   {send/recvmsg()}      {set/getsockopt()}          │    tlshd    │
   [CMSG handshake_info] [SOCKOPT_CRYPTO_SECRET]     └─────────────┘
                         [SOCKOPT_TRANSPORT_PARAM_EXT]    │   ^
                │ ^                  │ ^                  │   │
  Userspace     │ │                  │ │                  │   │
  ──────────────│─│──────────────────│─│──────────────────│───│────────
  Kernel        │ │                  │ │                  │   │
                v │                  v │                  v   │
  ┌──────────────────────────────────────────┐       ┌─────────────┐
  │ socket (IPPROTO_QUIC) |     protocol     │<──┐   │ handshake   │
  ├──────────────────────────────────────────┤   │   │netlink APIs │
  │ stream | connid | cong  | path  | timer  │   │   └─────────────┘
  ├──────────────────────────────────────────┤   │      │       │
  │  packet  |  frame  |  crypto  |  pnmap   │   │   ┌─────┐ ┌─────┐
  ├──────────────────────────────────────────┤   │   │     │ │     │
  │        input       |       output        │   │───│ SMB │ │ NFS │...
  ├──────────────────────────────────────────┤   │   │     │ │     │
  │                UDP tunnels               │   │   └─────┘ └─────┘
  └──────────────────────────────────────────┘   └──────┴───────┘

- User Data Architecture:

  ┌──────┐  ┌──────┐
  │ APP1 │  │ APP2 │ ...
  └──────┘  └──────┘
   {send/recvmsg()}      {set/getsockopt()}
   [CMSG stream_info]    [SOCKOPT_KEY_UPDATE]
                         [SOCKOPT_CONNECTION_MIGRATION]
                         [SOCKOPT_STREAM_OPEN/RESET/STOP_SENDING]
                │ ^                  │ ^
  Userspace     │ │                  │ │
  ──────────────│─│──────────────────│─│────────────────────────
  Kernel        │ │                  │ │
                v │                  v │
  ┌──────────────────────────────────────────┐
  │ socket (IPPROTO_QUIC) |     protocol     │<──┐{kernel_send/recvmsg()}
  ├──────────────────────────────────────────┤   │{kernel_set/getsockopt()}
  │ stream | connid | cong  | path  | timer  │   │
  ├──────────────────────────────────────────┤   │
  │  packet  |  frame  |  crypto  |  pnmap   │   │   ┌─────┐ ┌─────┐
  ├──────────────────────────────────────────┤   │   │     │ │     │
  │        input       |       output        │   │───│ SMB │ │ NFS │...
  ├──────────────────────────────────────────┤   │   │     │ │     │
  │                UDP tunnels               │   │   └─────┘ └─────┘
  └──────────────────────────────────────────┘   └──────┴───────┘

Usage
=====

This implementation supports a mapping of QUIC into sockets APIs. Similar
to TCP and SCTP, a typical Server and Client use the following system call
sequence to communicate:

    Client                             Server
  ──────────────────────────────────────────────────────────────────────
  sockfd = socket(IPPROTO_QUIC)      listenfd = socket(IPPROTO_QUIC)
  bind(sockfd)                       bind(listenfd)
                                     listen(listenfd)
  connect(sockfd)
  quic_client_handshake(sockfd)
                                     sockfd = accecpt(listenfd)
                                     quic_server_handshake(sockfd, cert)

  sendmsg(sockfd)                    recvmsg(sockfd)
  close(sockfd)                      close(sockfd)
                                     close(listenfd)

Please note that quic_client_handshake() and quic_server_handshake()
functions are currently sourced from libquic [3]. These functions are
responsible for receiving and processing the raw TLS handshake messages
until the completion of the handshake process.

For utilization by kernel consumers, it is essential to have tlshd
service [2] installed and running in userspace. This service receives
and manages kernel handshake requests for kernel sockets. In the kernel,
the APIs closely resemble those used in userspace:

    Client                             Server
  ────────────────────────────────────────────────────────────────────────
  __sock_create(IPPROTO_QUIC, &sock)  __sock_create(IPPROTO_QUIC, &sock)
  kernel_bind(sock)                   kernel_bind(sock)
                                      kernel_listen(sock)
  kernel_connect(sock)
  tls_client_hello_x509(args:{sock})
                                      kernel_accept(sock, &newsock)
                                      tls_server_hello_x509(args:{newsock})

  kernel_sendmsg(sock)                kernel_recvmsg(newsock)
  sock_release(sock)                  sock_release(newsock)
                                      sock_release(sock)

Please be aware that tls_client_hello_x509() and tls_server_hello_x509()
are APIs from net/handshake/. They are used to dispatch the handshake
request to the userspace tlshd service and subsequently block until the
handshake process is completed.

The QUIC module is currently labeled as "EXPERIMENTAL".

[1] https://datatracker.ietf.org/doc/draft-lxin-quic-socket-apis
[2] https://github.com/oracle/ktls-utils
[3] https://github.com/lxin/quic
