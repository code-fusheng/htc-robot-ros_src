#!/bin/bash
###
 # @Author: code-fusheng 2561035977@qq.com
 # @Date: 2023-09-22 10:15:08
 # @LastEditors: code-fusheng 2561035977@qq.com
 # @LastEditTime: 2023-09-25 09:45:07
 # @FilePath: /src/sync_2_mac.sh
 # @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
### 

rsync -avz --progress . fusheng@192.168.1.128:/Users/fusheng/WorkSpace/CompanyWork/work-fusheng/robot-pro/htc-robot-ros_ws/src