#ifndef EXO_NODE_H
#define EXO_NODE_H

/*
 * Stable node IDs used by the inter-node protocol.
 *
 * Keep these values synchronized with CONFIG_EXO_NODE_ID in each app's
 * prj.conf. Node ID 5 is intentionally left available for future use.
 */
#define EXO_NODE_ESB_RECEIVER 1U
#define EXO_NODE_ESB_FOOT_LEFT 2U
#define EXO_NODE_ESB_FOOT_RIGHT 3U
#define EXO_NODE_NUS_BRIDGE 4U
#define EXO_NODE_RESERVED 5U
#define EXO_NODE_BLE_RECEIVER 6U

#define EXO_NODE_ID_MIN EXO_NODE_ESB_RECEIVER
#define EXO_NODE_ID_MAX EXO_NODE_BLE_RECEIVER

#define EXO_THIS_NODE_ID CONFIG_EXO_NODE_ID
#define EXO_THIS_NODE_IS(node_id) (CONFIG_EXO_NODE_ID == (node_id))

#endif /* EXO_NODE_H */
