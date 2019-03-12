src     = []

component = aos_board_component('win32host', 'windows', src)

# Define the default component testcase set for this board
testcases = Split('''
    test/testcase/basic_test
    test/testcase/framework/alink_test
    test/testcase/middleware/uagent/uota_test
    test/testcase/framework/mqtt_test
    test/testcase/network/netmgr_test
    test/testcase/framework/wifi_hal_test
    test/testcase/kernel/modules/fatfs_test
    test/testcase/kernel/modules/kv_test
    test/testcase/kernel/rhino_test
    test/testcase/osal/aos/aos_test
    test/testcase/kernel/vfs_test
    test/testcase/network/protocols/umesh_test
    test/testcase/security/alicrypto_test
    test/testcase/security/id2_test
    test/testcase/security/tls_test
    test/testcase/utility/cjson_test
''')

component.set_global_testcases(testcases)
aos_global_config.set('MESHLOWPOWER',1)
