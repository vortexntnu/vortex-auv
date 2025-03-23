#include "lifecycle_manager/lifecycle_manager.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                R"(
	Millennium Falcon
                           ____          ____
                          / --.|        |.-- \
                         /,---||        ||---.\
                        //    ||        ||    \\
                       //     ||        ||     \\
                      // __   ||        ||   __ \\
                     //,'  `. ||        || ,'||`.\\
                    //( \`--|)||        ||( \'|  )\\
                   //  `.-_,' ||________|| `.-_,'  \\
                  //        | ||   _|   ||          \\
                 //         | || | .. | ||           \\
                //          | || | :: | ||            \\
               /    __     888|| |  _ | ||        __    \
              /   ,'|_'. _ 888|| |  | | ||  _   ,'-.`.   \
             /   ( ,-._ | |888|| | -| | || |_| (   |==)   \
            /     `O__,'|_|8@ || | [| | |@ |_|  `._|,'     \
           /            __..|--| | )| | |--|_|__            \           ,==.
          /      __,--''      |  |  | |  |      ``--.__      \         //[]\\
         /    ,::::'          |-=|  | |  |::::..       `-.    \       //||||\\
        /  ,-':::::       ..::|  | /| |  |::::::::        `-.  \     ||,'  `.||
       / ,'    ''       ::::::|  | \| |  |  ''::::     oo    `. \    |--------|
      /,'           ,'\ :::'''|] |  | |  |   ....    o8888o    `.\   |:::_[[[[]
     /'           ,'   )      |> |  | | [|  ::::::     88888  ..  \  |    |   ]
    /           ,'     |      |] |=_| | [|  '::::'oo     8   ::::  \-'    |---|
   .'     o   ,'    \_/       | :| \| |.[|      o8888o        ::;-'      ,'   |
  ,'     o88,'   `._/:\|      | :|  | |: |     88888888      ,-'  \  _,-'     |
  |     8888 ``--'|::::|      |_:|  | |:_|       8888     ,-'   \ ,-'   .::_,-'
 ,'    8888      ''--""      || || [| || ||            ,-'    _,-'   \ _::'
 |                           || |_,--._| ||         ,-'  \ ,-'     _,-'
 |                           |__(______)__|      ,-'   _,-' .::_,-'   |
|'                  88o     =====,,--..=====  .-'   ,-' \  _::'       `|
|                  888888o    ,-'\    /`-.     \ \-' . _,-'            |
`---._____        888888888 ,' / [HHHH[=====-   \ |_::'       _____.---'
  __|_____`=======._____   /   \ [HHHH[=====-    `'__.======='_____|__
 {I |  _______________ || /    /\/____\/\:.  \ || _______________  | I}
 [I |  :___]__[_______ ||(    /   _.._   \:.  )|| ______]___[___:  | I]
 {I_|____________o8o___|| \  /:  /\  /\   \: / ||__________________|_I}
    |_____,------'         \/:'  \ \/ /    \/:..   88`-------._____|
.---'   ::::::              `.   [||||]   ,'  '::..                `---.
|          '::..::        ,'  `-.      ,-'  `.  ':'                    |
|.        .:::''        ,' __    ``--''    __ `.     .::.             ,|
 |        '''         ,' ,'||`.    __    ,'||`. `.    '::::.          |
 |                  ,'  (||||||) ,'||`. (||||||) 88.    '::::.        |
 `.               ,'     `.||,' (||||||) `.||,' 88888.    ':'        ,'
  |             ,'      __       `.||,'       __  88  `.             |
  `.          ,'      ,'||`.       __       ,'||`.      `.          ,'
   `.       ,'       (||||||)    ,'||`.    (||||||)       `.       ,'
     \    ,'          `.||,'    (||||||)    `.||,'          `.    /
      `.  |                      `.||,'                      |  ,'
        `-|                                                  |-'
          `.   ,'.                                    .`.   ,'
            `-:,'o88o ,/                        \.     `.;-'
 Lifecycle     `-888 //      /     ||ooo  \ oo88 \\ __,-'   Cyprian
  -Manager          `--..__ /'     ||888  `\ oo8;--'
                           ```-----''-----''''
    )");
    auto node = std::make_shared<lifecycle_manager::LifecycleManager>();
    rclcpp::spin(node) rclcpp::shutdown();

    return 0;
}
