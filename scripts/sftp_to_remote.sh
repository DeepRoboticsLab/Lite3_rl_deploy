#!/usr/bin/expect
#exit

# set ip "192.168.2.1"
# set username "user"
# set passwd "123456"

set ip "192.168.2.1"
set username "ysc"
set passwd "'"

set send_policy "1"
set policy_name "policy_6_12.onnx"

if { "$send_policy" == "1" } {
  puts "send policy $policy_name to remote"
  spawn scp ./policy/Himloco/$policy_name  $username@$ip:/home/$username/rl_deploy/policy/Himloco/$policy_name
  expect {
    "密码："
          {
            send "$passwd\n"
          }
    "pass"
          {
            send "$passwd\n"
          }
    "yes/no"
          {
            sleep 5
            send_user "send yes"
            send "yes\n"
          }
    eof
      {
          sleep 5
          send_user "eof\n"
      }
  }
  set timeout 3000
  send "exit\r"
  expect eof
} else {
  puts "don't send policy to remote"
}

# /usr/bin/expect<<EOF

spawn scp build/rl_deploy  $username@$ip:/home/$username/rl_deploy/bin/rl_deploy
expect {
  "密码："
        {
          send "$passwd\n"
        }
   "pass"
        {
          send "$passwd\n"
        }
   "yes/no"
        {
          sleep 5
          send_user "send yes"
          send "yes\n"
        }
   eof
    {
        sleep 5
        send_user "eof\n"
    }
}
set timeout 3000
send "exit\r"
expect eof

# EOF
