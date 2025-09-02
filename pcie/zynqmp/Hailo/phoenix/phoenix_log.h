#ifndef PHOENIX_LOG
#define PHOENIX_LOG

#include <stdio.h>

#define dev_notice(dev, fmt, ...) (printf("hailo (dev_notice): " fmt, ##__VA_ARGS__))
#define dev_err(dev, fmt, ...) (printf("hailo (dev_err): " fmt, ##__VA_ARGS__))
#define pr_err(fmt, ...)     (printf("hailo (pr_err): " fmt, ##__VA_ARGS__))

#define hailo_emerg(board, fmt, ...)  printf("hailo (emerg):" fmt, ##__VA_ARGS__)
#define hailo_alert(board, fmt, ...)  printf("hailo (alert):" fmt, ##__VA_ARGS__)
#define hailo_crit(board, fmt, ...)   printf("hailo (crit):" fmt, ##__VA_ARGS__)
#define hailo_err(board, fmt, ...)    printf("hailo (err):" fmt, ##__VA_ARGS__)
#define hailo_warn(board, fmt, ...)   printf("hailo (warn):" fmt, ##__VA_ARGS__)
#define hailo_notice(board, fmt, ...) printf("hailo (notice):" fmt, ##__VA_ARGS__)
#define hailo_info(board, fmt, ...)   printf("hailo (info):" fmt, ##__VA_ARGS__)
#define hailo_dbg(board, fmt, ...)    printf("hailo (dbg):" fmt, ##__VA_ARGS__)

#define hailo_dev_emerg(dev, fmt, ...)  printf("hailo-dev (emerg): " fmt, ##__VA_ARGS__);
#define hailo_dev_alert(dev, fmt, ...)  printf("hailo-dev (aler): " fmt, ##__VA_ARGS__);
#define hailo_dev_crit(dev, fmt, ...)   printf("hailo-dev (crit): " fmt, ##__VA_ARGS__);
#define hailo_dev_err(dev, fmt, ...)    printf("hailo-dev (err): " fmt, ##__VA_ARGS__);
#define hailo_dev_warn(dev, fmt, ...)   printf("hailo-dev (warn): " fmt, ##__VA_ARGS__);
#define hailo_dev_notice(dev, fmt, ...) printf("hailo-dev (notice): " fmt, ##__VA_ARGS__);
#define hailo_dev_info(dev, fmt, ...)   printf("hailo-dev (info): " fmt, ##__VA_ARGS__);
#define hailo_dev_dbg(dev, fmt, ...)    printf("hailo-dev (dbg): " fmt, ##__VA_ARGS__);

#define pci_notice(dev, fmt, ...) printf("pci (notice): " fmt, ##__VA_ARGS__);
#define pci_err(dev, fmt, ...)    printf("pci (err): " fmt, ##__VA_ARGS__);
#endif
