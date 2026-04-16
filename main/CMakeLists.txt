idf_component_register(SRCS "main.cpp"
                       PRIV_REQUIRES nvs_flash esp_timer esp_wifi esp_event esp_netif
                                    esp_http_server json
                       EMBED_TXTFILES "bms_ui.html"
                       INCLUDE_DIRS "")
