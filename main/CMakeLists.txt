if(IDF_VERSION_MAJOR GREATER_EQUAL 4)
    idf_component_register(SRC_DIRS src
        REQUIRES esp_hw_support mbedtls log driver nvs_flash esp_event esp_wifi esp32-button app_hap_setup_payload esp_hap_apple_profiles esp_hap_core esp_hap_platform esp_system
        INCLUDE_DIRS include)
else()
    set(COMPONENT_SRCDIRS src)
    set(COMPONENT_ADD_INCLUDEDIRS include)
    set(COMPONENT_REQUIRES esp_hw_support mbedtls log driver nvs_flash esp_event esp_wifi esp32-button app_hap_setup_payload esp_hap_apple_profiles esp_hap_core esp_hap_platform esp_system)
    register_component()
endif()
