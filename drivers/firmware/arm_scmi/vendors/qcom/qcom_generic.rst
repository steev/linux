.. SPDX-License-Identifier: GPL-2.0
.. include:: <isonum.txt>

===============================================================================
QCOM System Control and Management Interface(SCMI) Vendor Protocols Extension
===============================================================================

:Copyright: |copy| 2024, Qualcomm Innovation Center, Inc. All rights reserved.

:Author: Sibi Sankar <quic_sibis@quicinc.com>

SCMI_GENERIC: System Control and Management Interface QCOM Generic Vendor Protocol
==================================================================================

This protocol is intended as a generic way of exposing a number of Qualcomm
SoC specific features through a mixture of pre-determined algorithm string and
param_id pairs hosted on the SCMI controller. It implements an interface compliant
with the Arm SCMI Specification with additional vendor specific commands as
detailed below.

Commands:
_________

PROTOCOL_VERSION
~~~~~~~~~~~~~~~~

message_id: 0x0
protocol_id: 0x80

+---------------+--------------------------------------------------------------+
|Return values                                                                 |
+---------------+--------------------------------------------------------------+
|Name           |Description                                                   |
+---------------+--------------------------------------------------------------+
|int32 status   |See ARM SCMI Specification for status code definitions.       |
+---------------+--------------------------------------------------------------+
|uint32 version |For this revision of the specification, this value must be    |
|               |0x10000.                                                      |
+---------------+--------------------------------------------------------------+

PROTOCOL_ATTRIBUTES
~~~~~~~~~~~~~~~~~~~

message_id: 0x1
protocol_id: 0x80

+---------------+--------------------------------------------------------------+
|Return values                                                                 |
+------------------+-----------------------------------------------------------+
|Name              |Description                                                |
+------------------+-----------------------------------------------------------+
|int32 status      |See ARM SCMI Specification for status code definitions.    |
+------------------+-----------------------------------------------------------+
|uint32 attributes |Bits[31:16] Reserved, must be to 0.                        |
|                  |Bits[15:8] Number of agents in the system                  |
|                  |Bits[7:0] Number of vendor protocols in the system         |
+------------------+-----------------------------------------------------------+

PROTOCOL_MESSAGE_ATTRIBUTES
~~~~~~~~~~~~~~~~~~~~~~~~~~~

message_id: 0x2
protocol_id: 0x80

+---------------+--------------------------------------------------------------+
|Return values                                                                 |
+------------------+-----------------------------------------------------------+
|Name              |Description                                                |
+------------------+-----------------------------------------------------------+
|int32 status      |See ARM SCMI Specification for status code definitions.    |
+------------------+-----------------------------------------------------------+
|uint32 attributes |For all message id's the parameter has a value of 0.       |
+------------------+-----------------------------------------------------------+

QCOM_SCMI_SET_PARAM
~~~~~~~~~~~~~~~~~~~

message_id: 0x10
protocol_id: 0x80

+------------------+-----------------------------------------------------------+
|Parameters                                                                    |
+------------------+-----------------------------------------------------------+
|Name              |Description                                                |
+------------------+-----------------------------------------------------------+
|uint32 ext_id     |Reserved, must be zero.                                    |
+------------------+-----------------------------------------------------------+
|uint32 algo_low   |Lower 32-bit value of the algorithm string.                |
+------------------+-----------------------------------------------------------+
|uint32 algo_high  |Upper 32-bit value of the algorithm string.                |
+------------------+-----------------------------------------------------------+
|uint32 param_id   |Serves as the token message id for the algorithm string    |
|                  |and is used to set various parameters supported by it.     |
+------------------+-----------------------------------------------------------+
|uint32 buf[]      |Serves as the payload for the specified param_id and       |
|                  |algorithm string pair.                                     |
+------------------+-----------------------------------------------------------+
|Return values                                                                 |
+------------------+-----------------------------------------------------------+
|Name              |Description                                                |
+------------------+-----------------------------------------------------------+
|int32 status      |SUCCESS: if the param_id and buf[] is parsed successfully  |
|                  |by the chosen algorithm string.                            |
|                  |NOT_SUPPORTED: if the algorithm string does not have any   |
|                  |matches.                                                   |
|                  |INVALID_PARAMETERS: if the param_id and the buf[] passed   |
|                  |is rejected by the algorithm string.                       |
+------------------+-----------------------------------------------------------+

QCOM_SCMI_GET_PARAM
~~~~~~~~~~~~~~~~~~~

message_id: 0x11
protocol_id: 0x80

+------------------+-----------------------------------------------------------+
|Parameters                                                                    |
+------------------+-----------------------------------------------------------+
|Name              |Description                                                |
+------------------+-----------------------------------------------------------+
|uint32 ext_id     |Reserved, must be zero.                                    |
+------------------+-----------------------------------------------------------+
|uint32 algo_low   |Lower 32-bit value of the algorithm string.                |
+------------------+-----------------------------------------------------------+
|uint32 algo_high  |Upper 32-bit value of the algorithm string.                |
+------------------+-----------------------------------------------------------+
|uint32 param_id   |Serves as the token message id for the algorithm string.   |
+------------------+-----------------------------------------------------------+
|uint32 buf[]      |Serves as the payload and store of value for the specified |
|                  |param_id and algorithm string pair.                        |
+------------------+-----------------------------------------------------------+
|Return values                                                                 |
+------------------+-----------------------------------------------------------+
|Name              |Description                                                |
+------------------+-----------------------------------------------------------+
|int32 status      |SUCCESS: if the param_id and buf[] is parsed successfully  |
|                  |by the chosen algorithm string and the result is copied    |
|                  |into buf[].                                                |
|                  |NOT_SUPPORTED: if the algorithm string does not have any   |
|                  |matches.                                                   |
|                  |INVALID_PARAMETERS: if the param_id and the buf[] passed   |
|                  |is rejected by the algorithm string.                       |
+------------------+-----------------------------------------------------------+

QCOM_SCMI_START_ACTIVITY
~~~~~~~~~~~~~~~~~~~~~~~~

message_id: 0x12
protocol_id: 0x80

+------------------+-----------------------------------------------------------+
|Parameters                                                                    |
+------------------+-----------------------------------------------------------+
|Name              |Description                                                |
+------------------+-----------------------------------------------------------+
|uint32 ext_id     |Reserved, must be zero.                                    |
+------------------+-----------------------------------------------------------+
|uint32 algo_low   |Lower 32-bit value of the algorithm string.                |
+------------------+-----------------------------------------------------------+
|uint32 algo_high  |Upper 32-bit value of the algorithm string.                |
+------------------+-----------------------------------------------------------+
|uint32 param_id   |Serves as the token message id for the algorithm string    |
|                  |and is generally used to start the activity performed by   |
|                  |the algorithm string.                                      |
+------------------+-----------------------------------------------------------+
|uint32 buf[]      |Serves as the payload for the specified param_id and       |
|                  |algorithm string pair.                                     |
+------------------+-----------------------------------------------------------+
|Return values                                                                 |
+------------------+-----------------------------------------------------------+
|Name              |Description                                                |
+------------------+-----------------------------------------------------------+
|int32 status      |SUCCESS: if the activity performed by the algorithm string |
|                  |starts successfully.                                       |
|                  |NOT_SUPPORTED: if the algorithm string does not have any.  |
|                  |matches or if the activity is already running.             |
+------------------+-----------------------------------------------------------+

QCOM_SCMI_STOP_ACTIVITY
~~~~~~~~~~~~~~~~~~~~~~~

message_id: 0x13
protocol_id: 0x80

+------------------+-----------------------------------------------------------+
|Parameters                                                                    |
+------------------+-----------------------------------------------------------+
|Name              |Description                                                |
+------------------+-----------------------------------------------------------+
|uint32 ext_id     |Reserved, must be zero.                                    |
+------------------+-----------------------------------------------------------+
|uint32 algo_low   |Lower 32-bit value of the algorithm string.                |
+------------------+-----------------------------------------------------------+
|uint32 algo_high  |Upper 32-bit value of the algorithm string.                |
+------------------+-----------------------------------------------------------+
|uint32 param_id   |Serves as the token message id for the algorithm string    |
|                  |and is generally used to stop the activity performed by    |
|                  |the algorithm string.                                      |
+------------------+-----------------------------------------------------------+
|uint32 buf[]      |Serves as the payload for the specified param_id and       |
|                  |algorithm string pair.                                     |
+------------------+-----------------------------------------------------------+
|Return values                                                                 |
+------------------+-----------------------------------------------------------+
|Name              |Description                                                |
+------------------+-----------------------------------------------------------+
|int32 status      |SUCCESS: if the activity performed by the algorithm string |
|                  |stops successfully.                                        |
|                  |NOT_SUPPORTED: if the algorithm string does not have any   |
|                  |matches or if the activity isn't running.                  |
+------------------+-----------------------------------------------------------+
