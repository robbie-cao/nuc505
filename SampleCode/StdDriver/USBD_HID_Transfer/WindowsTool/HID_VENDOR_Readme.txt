*USBD Device has 32KB Buffer for the test
*Page size is 256 Byte & Sector size is 4KB
*Test Flow
 - Send Erase CMD to Erase Sector 0-7 (Erase to '0xFF')
 - Send Read CMD to Read page 0 - 127 (Window Tool will check the read back data - all are '0xFF')
 - Send Write CMD to Write page 0 - 127 (Pattern = index % 256; After Write Command, USBD will check write data - Pattern = index % 256)
 - Send Read CMD to Read page 0 - 127 (Window Tool will check the read back data - Pattern = index % 256)
 - Send Erase CMD to Erase Sector 1 (Erase to '0xFF')
 - Send Read CMD to Read page 0 - 127 (Window Tool will check the read back data - Only Sector 1 are '0xFF', other are Pattern = index % 256)
