#!/bin/sh 
 
#将目录下的Demo{i}_{str}_Raw文件夹复制cp_num份, 分别命名为Demo{i}_{str}1,...,Demo{i}_{str}{n}

cp_num=15

if [ -e Demo*_*_Raw ]; then
    path=$(find -name "Demo*_*_Raw")
    var=${path%_Raw}
    var=${var#*_}
    num=${path%_*_*}
    num=${num#*Demo}
    for ((i=1; i<=cp_num; i++)) do
        # 如果Demo*_$var$i文件夹存在, 则删除
        if [ -e Demo*_$var$i ]; then
            rm -rf Demo*_$var$i
        fi
        cp -r $path Demo${num}_$var$i
        cd Demo${num}_$var$i
        nohup matlab -nodisplay -nosplash -nodesktop < swarmlab_server.m &
        cd ../
    done
fi