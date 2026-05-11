% Digital Accuracy Testing for Phantom X Pincher
clear; clc;

% 1. Setup Environment
% Replace with your actual robot model function
robot = twinModelCollisions();  %[output:613a9fd3] %[output:9bce8b1d]
current_q_servo = [0, 0, 0, 0]; % Start at home

% Preallocate table for results
numPoints = 5;
results = table('Size', [numPoints, 6], ...
    'VariableTypes', {'double', 'string', 'double', 'double', 'double', 'double'}, ...
    'VariableNames', {'PointID', 'Status', 'Target_X', 'Target_Y', 'Target_Z', 'Euclidean_Error_cm'});

% 2. Define Workspace Bounds (Example bounds for Pincher)
% Adjust these based on your robot's reach
x_range = [10, 20];
y_range = [-10, 10];
z_range = [5, 20];

fprintf('--- Starting Digital Accuracy Test ---\n'); %[output:11f83741]

for i = 1:numPoints %[output:group:4e8dec88]
    % 1. Select a Random Point p
    tx = x_range(1) + (x_range(2)-x_range(1))*rand();
    ty = y_range(1) + (y_range(2)-y_range(1))*rand();
    tz = z_range(1) + (z_range(2)-z_range(1))*rand();
    tphi = 0; % Target horizontal orientation
    
    % 2. Execute findSolution (IK)
    % This returns SERVO angles
    q_sol_servo = findSolution(tx, ty, tz, tphi, current_q_servo); %[output:4438f3b3] %[output:537d690a] %[output:4e47cb5b] %[output:5790254f] %[output:24025866] %[output:31210e01] %[output:6dc1343e] %[output:13bea2b0] %[output:56312c0f] %[output:99cf3778] %[output:9cc9dba7] %[output:967a2ea7] %[output:1f88d1d5] %[output:0b308b16] %[output:1a160796] %[output:63b94e52] %[output:9967666c] %[output:6959f46c] %[output:3529e96c] %[output:75a5ce1d] %[output:4bfc8f15] %[output:684be882] %[output:6f52fba3] %[output:91e2a35d] %[output:1dd49090] %[output:439c8b18] %[output:105fdca7] %[output:2e21eab5] %[output:1e21923b] %[output:40116571] %[output:05cf67e1] %[output:9bf5e103] %[output:8e1154d2] %[output:48543f74] %[output:85beff54] %[output:493af745] %[output:53a1d25d] %[output:8fe170f1] %[output:1a95bce1] %[output:97558fc8] %[output:02cfedef] %[output:141c78bc] %[output:2357b1a5] %[output:5699c557] %[output:8fe10a6b] %[output:5df6bfcb] %[output:81107a51] %[output:4debf811] %[output:75bb4dac] %[output:0f28d802] %[output:36b91aed] %[output:2e5c80f6] %[output:9fa591af] %[output:54d1d387] %[output:9dbdb451] %[output:5de335de] %[output:0ea0a53e] %[output:2b8db592] %[output:2d7198bf] %[output:5a17262d] %[output:4e59ea1f] %[output:90cc843c] %[output:9da70263] %[output:036b9c4c] %[output:8c294288] %[output:03d49953] %[output:7e4e0c85] %[output:8445b02c] %[output:77c66393] %[output:01afe7b4] %[output:7826e9c7] %[output:5bcee489] %[output:4291d608] %[output:47ad3d85] %[output:62dac386] %[output:86ce5e5c] %[output:029986b4] %[output:988fa24f] %[output:9258dea6] %[output:54f6e8a3] %[output:038cc7fc] %[output:8477c6e2] %[output:491eabcf] %[output:446c805b] %[output:7fcff884] %[output:2070e667] %[output:38919a5a] %[output:58b5e69a] %[output:2e4c849d] %[output:57568b89] %[output:5cf5aaa8] %[output:53001f39] %[output:10fe00bc] %[output:8825e916] %[output:5a254d0e] %[output:821ed6a0] %[output:5fb536b7] %[output:7855523c] %[output:4dc9e45b] %[output:8f57d4a2] %[output:1c944459] %[output:441942a2] %[output:83f12078] %[output:87e45e15] %[output:7df611c5] %[output:65144212] %[output:02d396ac] %[output:56574af0] %[output:8631b60f] %[output:5642c151] %[output:7218fce4] %[output:50f51bb5] %[output:685ab8a1] %[output:6b32b072] %[output:340fba75] %[output:6559725d] %[output:39ce1abc] %[output:6d14f8ac] %[output:57a50137] %[output:9623ce1e] %[output:31236ef2] %[output:0e72845e] %[output:0756b7c9] %[output:068e9444] %[output:78190698] %[output:31e1cc15] %[output:4cc2ded4] %[output:2b95c261] %[output:7b2ce7af] %[output:7991f075] %[output:5be8fe73] %[output:99dd2005] %[output:3f0b224f] %[output:01ab6bfe] %[output:0fbf05eb] %[output:6f42648e] %[output:48371ccf] %[output:85273f93] %[output:613f19ab] %[output:5b6570ae] %[output:3ec9fcd0] %[output:070e8d65] %[output:190bdf8e] %[output:24dca4dc] %[output:61ee45a8] %[output:9c738e54] %[output:8a78d86e] %[output:99ae15e6] %[output:3b737185] %[output:8edf2a79] %[output:50ab692f] %[output:720f02c5] %[output:8fbccb35] %[output:8343aab5] %[output:248ec760] %[output:7285d011] %[output:4545bb3b] %[output:77037ef5] %[output:7c3e0f61] %[output:2b10ef3b] %[output:0674944d] %[output:35fdbe27] %[output:6a0e3831] %[output:3f8c7e5a] %[output:0d0d91fc] %[output:6ce46eba] %[output:8d41a097] %[output:2d9c53f6] %[output:5efc3e2d] %[output:0ba915e8] %[output:2cda1e07] %[output:6fdc9348] %[output:3a0e5f5e] %[output:508dbe18] %[output:7e4bdc6d] %[output:966aeaac] %[output:2285641f] %[output:04bbc9f4] %[output:20163c06] %[output:94161cc6] %[output:2b87748e] %[output:322c145b] %[output:0544297e] %[output:3632881d] %[output:4362039b] %[output:1e5905f1] %[output:09e181bf] %[output:4b84f894] %[output:1fae113e] %[output:33fe48c3] %[output:39e3cca6] %[output:00e0787e] %[output:957930df] %[output:4a82f48d] %[output:92f47414] %[output:666db04b] %[output:9e2d175e] %[output:31419dc9] %[output:50322ada] %[output:4f0bcdd4] %[output:1b7d3154] %[output:1b900d5b] %[output:25d6b7ad] %[output:3728a490] %[output:66439a15] %[output:807599d4] %[output:08d6492c] %[output:42a75de6] %[output:50993595] %[output:68b643c0] %[output:2eb3ade6] %[output:4f069244] %[output:5985aecd] %[output:4bf8d5a8] %[output:2823ffd7] %[output:6b51f973] %[output:2ea32a00] %[output:2d59fe3d] %[output:4876599d] %[output:715a3646] %[output:0ba2274c] %[output:0fc3e6cb] %[output:6aee1dbb] %[output:639a38a5] %[output:205c2293] %[output:0f143ff8] %[output:67991683] %[output:7777f258] %[output:68184768] %[output:12cc6905] %[output:2b4b4017] %[output:4e4ac21e] %[output:5ac44285] %[output:89e8fa70] %[output:43a33959] %[output:9aefa2f6] %[output:8d509813] %[output:5c261d14] %[output:9b9d2170] %[output:3ad89fe0] %[output:54ecd5d3] %[output:1d7daeb4] %[output:9ca35d62] %[output:5deadceb] %[output:7cce2f7c] %[output:67d26a4f] %[output:301a06ad] %[output:80579b33] %[output:7c435b7b] %[output:9dced76c] %[output:9076e793] %[output:27abb0d7] %[output:8da483e3] %[output:4da1c692] %[output:328d0d88] %[output:39eca6bc] %[output:30304739] %[output:17dc9e93] %[output:46b7947f] %[output:4a66b910] %[output:07633ccd] %[output:6d0a078b] %[output:549d5656] %[output:4b97b412] %[output:9f6cc7cd] %[output:31225d7e] %[output:0d582778] %[output:4436be64] %[output:27925c09] %[output:799fc702] %[output:986744e0] %[output:90caba84] %[output:21e92d60] %[output:8642b4a8] %[output:81fbb978] %[output:1080652f] %[output:214d7a30] %[output:0cdddca9] %[output:695ceacf] %[output:4f15bee5] %[output:3c134bbb] %[output:7c7b734b] %[output:9c6dba03] %[output:127c6a40] %[output:4475862b] %[output:9f7ef8c5] %[output:624f897f] %[output:501488c9] %[output:06c32754] %[output:51becd18] %[output:81858d7b] %[output:80593cca] %[output:73331113] %[output:3d0ca3ba] %[output:60e31de1] %[output:686e871b] %[output:227d30b2] %[output:85f63bda] %[output:417f48b2] %[output:93d5371c] %[output:944a083b] %[output:60ad64db] %[output:37814080] %[output:61ce2d88] %[output:199f41ad] %[output:7d4356ee] %[output:3db4e0a6] %[output:877c6e40] %[output:1439877a] %[output:8312707e] %[output:26c9c83c] %[output:811efe7a] %[output:712bcbf8] %[output:78096c2e] %[output:060b373c] %[output:003114eb] %[output:996b5f8b] %[output:6c51c2dd] %[output:27953ede] %[output:811f9b88] %[output:6b6b40e1] %[output:25cb60e9] %[output:125f006d] %[output:5181c446] %[output:74cec857] %[output:86074b88] %[output:52320b40] %[output:3970eaa0] %[output:13559ddd] %[output:16686fbd] %[output:49125c6a] %[output:4a489ee5] %[output:84194810] %[output:2bae06ee] %[output:2b0070c6] %[output:1009de1a] %[output:345da921] %[output:6d2346da] %[output:46a51e05] %[output:83cc6fd8] %[output:76fd92bf] %[output:9f17726c] %[output:5dc97d6a] %[output:8cff5c49] %[output:30484b94] %[output:4bcb2f44] %[output:26dbf637] %[output:7984c169] %[output:05ab8a1a] %[output:4ff3500e] %[output:9d9caef1] %[output:9a6441b8] %[output:154b906a] %[output:4631a8a5] %[output:823e145b] %[output:210f38a3] %[output:71517938] %[output:32e53701] %[output:5f028fd2] %[output:784a7470] %[output:5b142b47] %[output:383d21ac] %[output:4b561159] %[output:41072423] %[output:09c067f7] %[output:20833e2e] %[output:59930ed3] %[output:2fd98776] %[output:08416f43] %[output:980362e8] %[output:0c01d8c2] %[output:3c1f76d4] %[output:184c2160] %[output:8ee73ffd] %[output:553cfd90] %[output:0c1c2d77] %[output:56559ddc] %[output:023f0c46] %[output:47217906] %[output:6df9e6af] %[output:5209e967] %[output:58c3ab37] %[output:6eedc468] %[output:26e214ae] %[output:31785783] %[output:744e6f10] %[output:64a212c5] %[output:779ad319] %[output:2bc19145] %[output:614915dd] %[output:77e744f0] %[output:153f3023] %[output:940dafdb] %[output:696dc91f] %[output:0e1b7dbc] %[output:47bfed53] %[output:73e108c0] %[output:8f71f947] %[output:443ea231] %[output:11efe2c0] %[output:2bbfd8c2] %[output:1a4c3e9a] %[output:428ea01b] %[output:0beb50f7] %[output:71f23331] %[output:063998ee] %[output:91513a72] %[output:7c285963] %[output:76893583] %[output:6d63a2c2] %[output:9e9542e2] %[output:197b3236] %[output:6c40f92e] %[output:63d03b1f] %[output:3e9a86cd] %[output:36a9cc0a] %[output:6377140f] %[output:42432e62] %[output:2c0399eb] %[output:7e6a6f0d] %[output:3b4114b7] %[output:65b2e28e] %[output:3e9afc5a] %[output:44fdc403] %[output:583434a6] %[output:640ae954] %[output:7d025e61] %[output:6e4e7792] %[output:0765ff8c] %[output:1975e103] %[output:66056bf3] %[output:8c7e4e13] %[output:4858bcf7] %[output:5031efd8] %[output:856bc707] %[output:335b0472] %[output:8a427173] %[output:87b6e1fd] %[output:5aab2458] %[output:30df8686] %[output:0e09ec47] %[output:0cb25a8c] %[output:9f2f6297] %[output:4eaff959] %[output:08e9debd] %[output:851cd83a] %[output:7a4fac27] %[output:04a0fb8f] %[output:7f04257f] %[output:69ebcfb3] %[output:0012f11a] %[output:0228061d] %[output:741cc3da] %[output:300e45e8] %[output:069c9414] %[output:3810f2b5] %[output:67f6abb3] %[output:17869f30] %[output:9a1713c2] %[output:39735d11] %[output:6a58b044] %[output:7f182b26] %[output:4a2d3fca] %[output:55b4792b] %[output:2c4b6653] %[output:18a68154] %[output:14a3e7f6] %[output:272b6603] %[output:5437f39e] %[output:6ca396e1] %[output:955553b6] %[output:6aeb143c] %[output:87fbc40e] %[output:72421218] %[output:30a5c961] %[output:0b335d3b] %[output:1ea338ae] %[output:9f3c89ef] %[output:7369559b] %[output:37664408] %[output:31f8a759] %[output:02442cac] %[output:422aa11b] %[output:5ac60e92] %[output:12cb5a03] %[output:35145db0] %[output:4ae5161f] %[output:3f74eef9] %[output:70ec2e36] %[output:161d9179] %[output:6433933e] %[output:013a619f] %[output:106a7808] %[output:01a0195f] %[output:53f2d94f] %[output:079e6c63] %[output:09d311a1] %[output:7913f569] %[output:6e7ebaf0] %[output:4890b168] %[output:50819ce9] %[output:87266e87] %[output:46a7c832] %[output:9b71e740] %[output:2b5502f3] %[output:78e503d1] %[output:7545027d] %[output:7b79e544] %[output:6e028b9c] %[output:7a8e1a11] %[output:702718c6] %[output:45d87701] %[output:956cbd5b] %[output:9ed8a8ab] %[output:3339f626] %[output:15705851] %[output:48f58434] %[output:85ce893a] %[output:590f57ee] %[output:78f1f9e6] %[output:703e451d] %[output:6b6e9719] %[output:4c1ae632] %[output:2fd6107d] %[output:3153b192] %[output:13a91745] %[output:52b24bd6] %[output:487d2ea0] %[output:0c6442f9] %[output:5f222433] %[output:7eeccecb] %[output:38a44ac8] %[output:01b2e8eb] %[output:7b77e2c8] %[output:0bcba92b] %[output:41601794] %[output:2d2eb4e2] %[output:66c9e772] %[output:36da68cb] %[output:7c31febd] %[output:5d08b9d0] %[output:99a80038] %[output:988440ce] %[output:303ad614] %[output:776f6254] %[output:9712d56c] %[output:101b0733] %[output:2585d2a2] %[output:57a2a2a6] %[output:90bed341] %[output:68499a3d] %[output:9b9eb0c1] %[output:6d6d5a33] %[output:5c85939a] %[output:46e5af66] %[output:496bdc05] %[output:6eedf877] %[output:3ebb40ea] %[output:0da54c6b] %[output:3fa8eeeb] %[output:066c4b59] %[output:6c8111b9] %[output:6e314453] %[output:28228bbc] %[output:4cd8104f] %[output:3c660350] %[output:041ed7dc] %[output:74e07f22] %[output:1f431059] %[output:46d6c9ef] %[output:3672afa2] %[output:9c7940e9] %[output:6021cbb5] %[output:3b273279] %[output:3bb90c91] %[output:1f4ed58b] %[output:8c500948] %[output:8a1867e4] %[output:2b2bfccd] %[output:085213f8] %[output:2e06932f] %[output:733d011d] %[output:9e34a8b9] %[output:8f66d0e5] %[output:2aed37a1] %[output:5bc61391] %[output:2a03c4fc] %[output:8618e03e] %[output:5e352328] %[output:204dec44] %[output:97493724] %[output:17aa802e] %[output:906c637e] %[output:5693c099] %[output:6bd1f777] %[output:7fccaf56] %[output:489bd583] %[output:5098ec3b] %[output:853cdf55] %[output:90664767] %[output:953b8f18] %[output:47c563d7] %[output:82741542] %[output:921aad0b] %[output:472464d4] %[output:19d672be] %[output:50e64eff] %[output:36738764] %[output:7b951aee] %[output:7271fc34] %[output:2e2e3ea2] %[output:8b7fb3a5] %[output:78825d5e] %[output:4d0a0de7] %[output:4c4b68de] %[output:97a4b742] %[output:35bf34ad] %[output:68d7c888] %[output:3b8f724d] %[output:832cd5d1] %[output:71ddd7f8] %[output:222ecb2f] %[output:24f0fea7] %[output:80491036] %[output:491d6be6] %[output:23c1e855] %[output:926fea8c] %[output:3e5ab6a9] %[output:5c2b499d] %[output:805c2770] %[output:65608651] %[output:986ad192] %[output:12d17120] %[output:981844e9] %[output:5a3d508a] %[output:63583468] %[output:0fb03910] %[output:1ae3c5a3] %[output:64804d7a] %[output:5f2e5027] %[output:2223bcd9] %[output:7032a66f] %[output:2f12a9f7] %[output:597bee82] %[output:22b1400d] %[output:40bc9e8b] %[output:5a5f39e6] %[output:027e4fc6] %[output:24701a23] %[output:8822b499] %[output:0be2b954] %[output:45c00898] %[output:48d35ca1] %[output:47b5288b] %[output:414f972d] %[output:28cd6b82] %[output:370fbbd6] %[output:23473382] %[output:40aef911] %[output:2c0601da] %[output:91e40e51] %[output:5da1b0e8] %[output:34573ed2] %[output:1a184281] %[output:0c5584fb] %[output:74c2ec2b] %[output:018e2f8a] %[output:355f1e6e] %[output:6785b180] %[output:3ff71ff1] %[output:5df68c77] %[output:3e26defc] %[output:42fb4211] %[output:8bcc2cbd] %[output:38a55a7a] %[output:1c88b3a2] %[output:4486f182] %[output:09937f66] %[output:8035e616] %[output:6ebad528] %[output:53e24d6f] %[output:467e6a6e] %[output:4e75c452] %[output:5147f3c5] %[output:7145d6a7] %[output:36924aad] %[output:09ce2170] %[output:2d882e73] %[output:7c8c2477] %[output:21e02962] %[output:0b9c46f9] %[output:270875a0] %[output:8c7b415d] %[output:673f91c6] %[output:3aa67e7e] %[output:6a26c31f] %[output:047302c4] %[output:3651757d] %[output:2fd94782] %[output:9aee0727] %[output:866d64e7] %[output:79007169] %[output:3bc98fa0] %[output:506e9577] %[output:683338ea] %[output:986f4156] %[output:591a2a0f] %[output:25eb2987] %[output:712c4ea0] %[output:3c86b27c] %[output:87083bd3] %[output:3d03ddb8] %[output:9572a411] %[output:50a7c073] %[output:7c16d1af] %[output:76f35c5b] %[output:59c6ffae] %[output:167b6440] %[output:68b152c1] %[output:9d4e7ab7] %[output:25ea5cc8] %[output:46455e13] %[output:36225231] %[output:24faa382] %[output:6bd9d2c8] %[output:072dd645] %[output:56b335c8] %[output:1451f31b] %[output:6fbc1988] %[output:60b3415d] %[output:5d27b3f2] %[output:984f33ab] %[output:544ce237] %[output:2fd72dc4] %[output:7b831f70] %[output:8956f6d5] %[output:763148b4] %[output:719240bf] %[output:87bb3278] %[output:402a89e6] %[output:6ed34da6] %[output:3b8b77ad] %[output:639415ac] %[output:258a9a10] %[output:6c95da56] %[output:7fcb07ab] %[output:547b3ead] %[output:795999dd] %[output:834cc6ee] %[output:8653d882] %[output:4c97ecea] %[output:5391e9b0] %[output:2faa6255] %[output:0ea7a4af] %[output:6de8a45f] %[output:34ec755b] %[output:31b3859c] %[output:4b5ed953] %[output:2bd05ac9] %[output:83858fce] %[output:9be892ed] %[output:717ab206] %[output:026c2f62] %[output:7d368bcc] %[output:5382ea37] %[output:43a450bb] %[output:7cb710eb] %[output:6e8052f3] %[output:32b0dcf0] %[output:8f492cc5] %[output:0907e3a3] %[output:1eef8d8f] %[output:6efb82f5] %[output:278804c9] %[output:4f2a16cb] %[output:2fea4f76] %[output:6d9d4819] %[output:913c133c] %[output:0ea7c9c6] %[output:254df142] %[output:33e21062] %[output:43ce4daa] %[output:4728c101] %[output:3682ded8] %[output:4d2c0b66] %[output:4e2b82a5] %[output:216f571d] %[output:6e3f58ed] %[output:1b12a300] %[output:19fe0694] %[output:8cf5404a] %[output:86b07d85] %[output:336b4c70] %[output:9f55b8fd] %[output:8869b43d] %[output:92fbf951] %[output:097fa215] %[output:02f3c7db] %[output:7f9fc1dd] %[output:44fe1de9] %[output:6261f7a0] %[output:0ac06073] %[output:95a0ce1b] %[output:7a008307] %[output:413d7367] %[output:4d8b18cd] %[output:9421bcd4] %[output:479dad90] %[output:7184803a] %[output:4ada6a4d] %[output:783dd575] %[output:10815cdd] %[output:71ef2b32] %[output:35decdf0] %[output:4037e6f0] %[output:78a390e8] %[output:8d6e326d] %[output:482a5533] %[output:9f31f500] %[output:8350302e] %[output:8191a539] %[output:4ea64dc6] %[output:1caefc4b] %[output:4d0607ae] %[output:28902f9e] %[output:288b3e13] %[output:5e51b919] %[output:163c98af] %[output:38cb9e1b] %[output:81cac3e4] %[output:9569bfae] %[output:1bb951bf] %[output:375af20d] %[output:1a044e3a] %[output:683a46a1] %[output:26a2009f] %[output:4886aaff] %[output:59162c11] %[output:68532df0] %[output:99b877ed] %[output:761e70b4] %[output:927aa85e] %[output:7bdd1a88] %[output:5c96c7ea] %[output:9efeeecf] %[output:1889083c] %[output:4d2035ee] %[output:25d59388] %[output:1834c7ab] %[output:344f0533] %[output:46552c9c] %[output:73dd03e0] %[output:5ae8bb12] %[output:45c3e0b3] %[output:565ddfb2] %[output:6cdacbe3] %[output:698d541a] %[output:4ef7dccc] %[output:1cbff8a5] %[output:3cf7b960] %[output:71218cb0] %[output:902bbda7] %[output:3123c8a2] %[output:9dae6173] %[output:985ac666] %[output:7f12b045] %[output:4760906d] %[output:189aba86] %[output:45f05001] %[output:9f7cfc2e] %[output:8671d85d] %[output:836eb06e] %[output:28fd952e] %[output:20bf6899] %[output:1509b444] %[output:09f058bb] %[output:4a92558b] %[output:6757518d] %[output:1479d6e8] %[output:5201e7de] %[output:2c7a7912] %[output:92137c8e] %[output:893a0e47] %[output:13a51b29] %[output:17b16ecb] %[output:961bb7f6] %[output:17ceb9a0] %[output:77f75f4b] %[output:0ae0e47f] %[output:071894dc] %[output:9f5446f4] %[output:323c0278] %[output:81f5d579] %[output:549a4b6b] %[output:405298ea] %[output:42313766] %[output:76002630] %[output:35070a34] %[output:3863b032] %[output:6cb24a03] %[output:4b83099c] %[output:2002a054] %[output:460ddca9] %[output:2a4683b5] %[output:92af0465] %[output:7f33e94d] %[output:00d929fe] %[output:987a1e5d] %[output:7a05d1ab] %[output:56970a22] %[output:6ff27c96] %[output:43cd7971] %[output:7f740579] %[output:68561ec1] %[output:471c61ef] %[output:309dc404] %[output:66297509] %[output:11f7a08a] %[output:53586b0b] %[output:4a632fd0] %[output:3560f6fe] %[output:466f5f14] %[output:7817b20e] %[output:2ea18f6d] %[output:33e1252d] %[output:67b5031d] %[output:6f2d44e0] %[output:30cae853] %[output:9f1fb520] %[output:906c2c72] %[output:9d1008a4] %[output:90b4899e] %[output:074b7bda] %[output:53c52a78] %[output:91b7b4ac] %[output:157849e8] %[output:610f8645] %[output:4e48bccb] %[output:970d1739] %[output:9897ca3e] %[output:5e2e2644] %[output:2f757f7a] %[output:06b2c14a] %[output:5d2bf9a6] %[output:83960ba7] %[output:3f60d49f] %[output:71a2e2fd] %[output:9800d7d1] %[output:68eb1f6d] %[output:23f03cf6] %[output:2b549663] %[output:02c75646] %[output:5eae9f39] %[output:724db77c] %[output:6ff76f29] %[output:5fe2a7c6] %[output:6960f497] %[output:176be319] %[output:7a4159ac] %[output:179aff53] %[output:5d2c3ada] %[output:23dea989] %[output:0cd5c31c] %[output:115cca8d] %[output:5814c5fd] %[output:787e5d2b] %[output:519ad10e] %[output:3530ca9b] %[output:73b4c59c] %[output:49ccf3e2] %[output:9301b664] %[output:95a22f52] %[output:4b54e8df] %[output:9969b3b4] %[output:6d3961a7] %[output:5a001368] %[output:21a7e1d4] %[output:5be55dc8] %[output:7771dac7] %[output:07002513] %[output:4977646d] %[output:88de64b3] %[output:377e3f21] %[output:2b9cb781] %[output:6668d440] %[output:205dc45f] %[output:34673ccb] %[output:2998c2e9] %[output:3b2f714e] %[output:600ef699] %[output:501e18ca] %[output:878e9faf] %[output:8822df20] %[output:1f6b9cfa] %[output:0d769b68] %[output:979c9e7b] %[output:59d94e01] %[output:59b89d30] %[output:84f61951] %[output:4e09b430] %[output:4a081a98] %[output:960948b2] %[output:8ab9b1f2] %[output:1052a1ed] %[output:05dfb809] %[output:433db920] %[output:04ce6ba9] %[output:4ffdbece] %[output:66ba0994] %[output:8627e72b] %[output:2472061c] %[output:9e8867da] %[output:185a7833] %[output:20a02812] %[output:863e5b54] %[output:770b5385] %[output:4a8d0a75] %[output:91c9ae5f] %[output:03ac9bdf] %[output:11d5ad8c] %[output:6f2f4a13] %[output:97d449fd] %[output:6b80f0ba] %[output:1c7fd3eb] %[output:742ea4a2] %[output:2ac47630] %[output:13809a39] %[output:72d8b240] %[output:47b6fd3f] %[output:1a806885] %[output:38be4d0d] %[output:49d09cbf] %[output:3c62573b] %[output:03620ffb] %[output:115f148f] %[output:9d32477d] %[output:09f8fa4e] %[output:7c59034e] %[output:44565e58] %[output:9c0fd06d] %[output:5d19101f] %[output:327baba1] %[output:1d874fc0] %[output:4aa4d765] %[output:9f80adc1] %[output:9eab8c4a] %[output:7365b63a] %[output:41078543] %[output:374cf5ce] %[output:53758233] %[output:4bddca68] %[output:73abb451] %[output:8feca7b6] %[output:97dbfecc] %[output:97ada13b] %[output:1e3ccd4e] %[output:2ac27e94] %[output:4b0fc46e] %[output:00dd1a49] %[output:3db97374] %[output:4c4ac35f] %[output:5862122e] %[output:63a542d4] %[output:9345cd27] %[output:645243d9] %[output:847f62b3] %[output:5a14c495] %[output:30b46f26] %[output:47b1e694] %[output:713e796d] %[output:7be612fd] %[output:7f12eda3] %[output:367de070] %[output:43a0396d] %[output:39ac0fcb] %[output:6269235e] %[output:65110a05] %[output:1574d15c] %[output:3c2b8043] %[output:43f5144c] %[output:83acd050] %[output:2d2d4933] %[output:99184f82] %[output:15b52179] %[output:4abe06d9] %[output:69d61557] %[output:8ec2759e] %[output:1637bcd6] %[output:9ec03362] %[output:2b44aa03] %[output:20060263] %[output:8343756d] %[output:8abbcdd9] %[output:50c563b6] %[output:550aa568] %[output:53e686d3] %[output:304bbd15] %[output:00487bcb] %[output:5f4ea63d] %[output:8563c579] %[output:673ca518] %[output:9fea9564] %[output:57745a8a] %[output:5f5cc2b4] %[output:6a2c3927] %[output:48b56181] %[output:0c972d00] %[output:8cb034b6] %[output:703294b4] %[output:28bdc972] %[output:67460edd] %[output:4e54c4e9] %[output:4a620c16] %[output:76b94fa5] %[output:48bd1345] %[output:096fe590] %[output:59f0065e] %[output:4ff04843] %[output:894ff500] %[output:0cabd540] %[output:2d20d499]
    
    results.PointID(i) = i;
    results.Target_X(i) = tx;
    results.Target_Y(i) = ty;
    results.Target_Z(i) = tz;
    
    if isempty(q_sol_servo)
        results.Status(i) = "OUT OF REACH";
        results.Euclidean_Error_cm(i) = NaN;
        continue;
    end
    
    % 3. "Measure" the actual location using FK (The Ground Truth)
    % Convert servo solution back to DH for the rigidBodyTree
    q_sol_dh = servo2dh(q_sol_servo);
    tform = getTransform(robot, q_sol_dh, 'link4');
    actual_pos = tform(1:3, 4); % Extract X, Y, Z from transformation matrix
    
    % 4. Compute Euclidean Error
    error = sqrt(sum(([tx; ty; tz] - actual_pos).^2));
    
    results.Status(i) = "SUCCESS";
    results.Euclidean_Error_cm(i) = error;
    
    % Update current position for next iteration
    current_q_servo = q_sol_servo;
end %[output:group:4e8dec88]

% 5. Display the Results
disp(results); %[output:048b7d4c]
fprintf('Mean Absolute Digital Error: %.4f cm\n', mean(results.Euclidean_Error_cm, 'omitnan')); %[output:0adc5e09]

if mean(results.Euclidean_Error_cm, 'omitnan') > 0.01 %[output:group:2cf03c15]
    fprintf('\nWARNING: Your digital error is high. Check your Z-offsets or DH parameters.\n'); %[output:21a4b44d]
end %[output:group:2cf03c15]

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright"}
%---
%[output:613a9fd3]
%   data: {"dataType":"text","outputData":{"text":"--------------------\nRobot: (4 bodies)\n\n Idx    Body Name    Joint Name    Joint Type    Parent Name(Idx)   Children Name(s)\n ---    ---------    ----------    ----------    ----------------   ----------------\n   1        link1        joint1      revolute             base(0)   link2(2)  \n   2        link2        joint2      revolute            link1(1)   link3(3)  \n   3        link3        joint3      revolute            link2(2)   link4(4)  \n   4        link4        joint4      revolute            link3(3)   \n--------------------\n","truncated":false}}
%---
%[output:11f83741]
%   data: {"dataType":"text","outputData":{"text":"--- Starting Digital Accuracy Test ---\n","truncated":false}}
%---
%[output:4438f3b3]
%   data: {"dataType":"text","outputData":{"text":"--------------------\nRobot: (4 bodies)\n\n Idx    Body Name    Joint Name    Joint Type    Parent Name(Idx)   Children Name(s)\n ---    ---------    ----------    ----------    ----------------   ----------------\n   1        link1        joint1      revolute             base(0)   link2(2)  \n   2        link2        joint2      revolute            link1(1)   link3(3)  \n   3        link3        joint3      revolute            link2(2)   link4(4)  \n   4        link4        joint4      revolute            link3(3)   \n--------------------\n","truncated":false}}
%---
%[output:537d690a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4e47cb5b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5790254f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:24025866]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:31210e01]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6dc1343e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:13bea2b0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:56312c0f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:99cf3778]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9cc9dba7]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:967a2ea7]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1f88d1d5]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0b308b16]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1a160796]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:63b94e52]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9967666c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6959f46c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3529e96c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:75a5ce1d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4bfc8f15]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:684be882]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6f52fba3]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:91e2a35d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1dd49090]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:439c8b18]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:105fdca7]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2e21eab5]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1e21923b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:40116571]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:05cf67e1]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9bf5e103]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8e1154d2]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:48543f74]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:85beff54]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:493af745]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:53a1d25d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8fe170f1]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1a95bce1]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:97558fc8]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:02cfedef]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:141c78bc]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2357b1a5]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5699c557]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8fe10a6b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5df6bfcb]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:81107a51]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4debf811]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:75bb4dac]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0f28d802]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:36b91aed]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2e5c80f6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9fa591af]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:54d1d387]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9dbdb451]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5de335de]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0ea0a53e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2b8db592]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2d7198bf]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5a17262d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4e59ea1f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:90cc843c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9da70263]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:036b9c4c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8c294288]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:03d49953]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7e4e0c85]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8445b02c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:77c66393]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:01afe7b4]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7826e9c7]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5bcee489]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4291d608]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:47ad3d85]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:62dac386]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:86ce5e5c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:029986b4]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:988fa24f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9258dea6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:54f6e8a3]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:038cc7fc]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8477c6e2]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:491eabcf]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:446c805b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7fcff884]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2070e667]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:38919a5a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:58b5e69a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2e4c849d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:57568b89]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5cf5aaa8]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:53001f39]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:10fe00bc]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8825e916]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5a254d0e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:821ed6a0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5fb536b7]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7855523c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4dc9e45b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8f57d4a2]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1c944459]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:441942a2]
%   data: {"dataType":"text","outputData":{"text":"--------------------\nRobot: (4 bodies)\n\n Idx    Body Name    Joint Name    Joint Type    Parent Name(Idx)   Children Name(s)\n ---    ---------    ----------    ----------    ----------------   ----------------\n   1        link1        joint1      revolute             base(0)   link2(2)  \n   2        link2        joint2      revolute            link1(1)   link3(3)  \n   3        link3        joint3      revolute            link2(2)   link4(4)  \n   4        link4        joint4      revolute            link3(3)   \n--------------------\n","truncated":false}}
%---
%[output:83f12078]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:87e45e15]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7df611c5]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:65144212]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:02d396ac]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:56574af0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8631b60f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5642c151]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7218fce4]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:50f51bb5]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:685ab8a1]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6b32b072]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:340fba75]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6559725d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:39ce1abc]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6d14f8ac]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:57a50137]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9623ce1e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:31236ef2]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0e72845e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0756b7c9]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:068e9444]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:78190698]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:31e1cc15]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4cc2ded4]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2b95c261]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7b2ce7af]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7991f075]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5be8fe73]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:99dd2005]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3f0b224f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:01ab6bfe]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0fbf05eb]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6f42648e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:48371ccf]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:85273f93]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:613f19ab]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5b6570ae]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3ec9fcd0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:070e8d65]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:190bdf8e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:24dca4dc]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:61ee45a8]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9c738e54]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8a78d86e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:99ae15e6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3b737185]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8edf2a79]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:50ab692f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:720f02c5]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8fbccb35]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8343aab5]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:248ec760]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7285d011]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4545bb3b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:77037ef5]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7c3e0f61]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2b10ef3b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0674944d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:35fdbe27]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6a0e3831]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3f8c7e5a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0d0d91fc]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6ce46eba]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8d41a097]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2d9c53f6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5efc3e2d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0ba915e8]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2cda1e07]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6fdc9348]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3a0e5f5e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:508dbe18]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7e4bdc6d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:966aeaac]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2285641f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:04bbc9f4]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:20163c06]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:94161cc6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2b87748e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:322c145b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0544297e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3632881d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4362039b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1e5905f1]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:09e181bf]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4b84f894]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1fae113e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:33fe48c3]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:39e3cca6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:00e0787e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:957930df]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4a82f48d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:92f47414]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:666db04b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9e2d175e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:31419dc9]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:50322ada]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4f0bcdd4]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1b7d3154]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1b900d5b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:25d6b7ad]
%   data: {"dataType":"text","outputData":{"text":"--------------------\nRobot: (4 bodies)\n\n Idx    Body Name    Joint Name    Joint Type    Parent Name(Idx)   Children Name(s)\n ---    ---------    ----------    ----------    ----------------   ----------------\n   1        link1        joint1      revolute             base(0)   link2(2)  \n   2        link2        joint2      revolute            link1(1)   link3(3)  \n   3        link3        joint3      revolute            link2(2)   link4(4)  \n   4        link4        joint4      revolute            link3(3)   \n--------------------\n","truncated":false}}
%---
%[output:3728a490]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:66439a15]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:807599d4]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:08d6492c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:42a75de6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:50993595]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:68b643c0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2eb3ade6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4f069244]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5985aecd]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4bf8d5a8]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2823ffd7]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6b51f973]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2ea32a00]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2d59fe3d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4876599d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:715a3646]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0ba2274c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0fc3e6cb]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6aee1dbb]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:639a38a5]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:205c2293]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0f143ff8]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:67991683]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7777f258]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:68184768]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:12cc6905]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2b4b4017]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4e4ac21e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5ac44285]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:89e8fa70]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:43a33959]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9aefa2f6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8d509813]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5c261d14]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9b9d2170]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3ad89fe0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:54ecd5d3]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1d7daeb4]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9ca35d62]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5deadceb]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7cce2f7c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:67d26a4f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:301a06ad]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:80579b33]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7c435b7b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9dced76c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9076e793]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:27abb0d7]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8da483e3]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4da1c692]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:328d0d88]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:39eca6bc]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:30304739]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:17dc9e93]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:46b7947f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4a66b910]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:07633ccd]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6d0a078b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:549d5656]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4b97b412]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9f6cc7cd]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:31225d7e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0d582778]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4436be64]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:27925c09]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:799fc702]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:986744e0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:90caba84]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:21e92d60]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8642b4a8]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:81fbb978]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1080652f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:214d7a30]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0cdddca9]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:695ceacf]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4f15bee5]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3c134bbb]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7c7b734b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9c6dba03]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:127c6a40]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4475862b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9f7ef8c5]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:624f897f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:501488c9]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:06c32754]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:51becd18]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:81858d7b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:80593cca]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:73331113]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3d0ca3ba]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:60e31de1]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:686e871b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:227d30b2]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:85f63bda]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:417f48b2]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:93d5371c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:944a083b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:60ad64db]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:37814080]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:61ce2d88]
%   data: {"dataType":"text","outputData":{"text":"--------------------\nRobot: (4 bodies)\n\n Idx    Body Name    Joint Name    Joint Type    Parent Name(Idx)   Children Name(s)\n ---    ---------    ----------    ----------    ----------------   ----------------\n   1        link1        joint1      revolute             base(0)   link2(2)  \n   2        link2        joint2      revolute            link1(1)   link3(3)  \n   3        link3        joint3      revolute            link2(2)   link4(4)  \n   4        link4        joint4      revolute            link3(3)   \n--------------------\n","truncated":false}}
%---
%[output:199f41ad]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7d4356ee]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3db4e0a6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:877c6e40]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1439877a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8312707e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:26c9c83c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:811efe7a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:712bcbf8]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:78096c2e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:060b373c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:003114eb]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:996b5f8b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6c51c2dd]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:27953ede]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:811f9b88]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6b6b40e1]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:25cb60e9]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:125f006d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5181c446]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:74cec857]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:86074b88]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:52320b40]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3970eaa0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:13559ddd]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:16686fbd]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:49125c6a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4a489ee5]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:84194810]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2bae06ee]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2b0070c6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1009de1a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:345da921]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6d2346da]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:46a51e05]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:83cc6fd8]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:76fd92bf]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9f17726c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5dc97d6a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8cff5c49]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:30484b94]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4bcb2f44]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:26dbf637]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7984c169]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:05ab8a1a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4ff3500e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9d9caef1]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9a6441b8]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:154b906a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4631a8a5]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:823e145b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:210f38a3]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:71517938]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:32e53701]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5f028fd2]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:784a7470]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5b142b47]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:383d21ac]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4b561159]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:41072423]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:09c067f7]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:20833e2e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:59930ed3]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2fd98776]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:08416f43]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:980362e8]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0c01d8c2]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3c1f76d4]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:184c2160]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8ee73ffd]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:553cfd90]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0c1c2d77]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:56559ddc]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:023f0c46]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:47217906]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6df9e6af]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5209e967]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:58c3ab37]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6eedc468]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:26e214ae]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:31785783]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:744e6f10]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:64a212c5]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:779ad319]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2bc19145]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:614915dd]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:77e744f0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:153f3023]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:940dafdb]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:696dc91f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0e1b7dbc]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:47bfed53]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:73e108c0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8f71f947]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:443ea231]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:11efe2c0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2bbfd8c2]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1a4c3e9a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:428ea01b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0beb50f7]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:71f23331]
%   data: {"dataType":"text","outputData":{"text":"--------------------\nRobot: (4 bodies)\n\n Idx    Body Name    Joint Name    Joint Type    Parent Name(Idx)   Children Name(s)\n ---    ---------    ----------    ----------    ----------------   ----------------\n   1        link1        joint1      revolute             base(0)   link2(2)  \n   2        link2        joint2      revolute            link1(1)   link3(3)  \n   3        link3        joint3      revolute            link2(2)   link4(4)  \n   4        link4        joint4      revolute            link3(3)   \n--------------------\n","truncated":false}}
%---
%[output:063998ee]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:91513a72]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7c285963]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:76893583]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6d63a2c2]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9e9542e2]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:197b3236]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6c40f92e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:63d03b1f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3e9a86cd]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:36a9cc0a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6377140f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:42432e62]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2c0399eb]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7e6a6f0d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3b4114b7]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:65b2e28e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3e9afc5a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:44fdc403]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:583434a6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:640ae954]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7d025e61]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6e4e7792]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0765ff8c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1975e103]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:66056bf3]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8c7e4e13]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4858bcf7]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5031efd8]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:856bc707]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:335b0472]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8a427173]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:87b6e1fd]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5aab2458]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:30df8686]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0e09ec47]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0cb25a8c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9f2f6297]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4eaff959]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:08e9debd]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:851cd83a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7a4fac27]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:04a0fb8f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7f04257f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:69ebcfb3]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0012f11a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0228061d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:741cc3da]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:300e45e8]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:069c9414]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3810f2b5]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:67f6abb3]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:17869f30]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9a1713c2]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:39735d11]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6a58b044]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7f182b26]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4a2d3fca]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:55b4792b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2c4b6653]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:18a68154]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:14a3e7f6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:272b6603]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5437f39e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6ca396e1]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:955553b6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6aeb143c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:87fbc40e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:72421218]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:30a5c961]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0b335d3b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1ea338ae]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9f3c89ef]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7369559b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:37664408]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:31f8a759]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:02442cac]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:422aa11b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5ac60e92]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:12cb5a03]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:35145db0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4ae5161f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3f74eef9]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:70ec2e36]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:161d9179]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6433933e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:013a619f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:106a7808]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:01a0195f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:53f2d94f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:079e6c63]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:09d311a1]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7913f569]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6e7ebaf0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4890b168]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:50819ce9]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:87266e87]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:46a7c832]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9b71e740]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2b5502f3]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:78e503d1]
%   data: {"dataType":"text","outputData":{"text":"--------------------\nRobot: (4 bodies)\n\n Idx    Body Name    Joint Name    Joint Type    Parent Name(Idx)   Children Name(s)\n ---    ---------    ----------    ----------    ----------------   ----------------\n   1        link1        joint1      revolute             base(0)   link2(2)  \n   2        link2        joint2      revolute            link1(1)   link3(3)  \n   3        link3        joint3      revolute            link2(2)   link4(4)  \n   4        link4        joint4      revolute            link3(3)   \n--------------------\n","truncated":false}}
%---
%[output:7545027d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7b79e544]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6e028b9c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7a8e1a11]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:702718c6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:45d87701]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:956cbd5b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9ed8a8ab]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3339f626]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:15705851]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:48f58434]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:85ce893a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:590f57ee]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:78f1f9e6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:703e451d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6b6e9719]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4c1ae632]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2fd6107d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3153b192]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:13a91745]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:52b24bd6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:487d2ea0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0c6442f9]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5f222433]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7eeccecb]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:38a44ac8]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:01b2e8eb]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7b77e2c8]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0bcba92b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:41601794]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2d2eb4e2]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:66c9e772]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:36da68cb]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7c31febd]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5d08b9d0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:99a80038]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:988440ce]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:303ad614]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:776f6254]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9712d56c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:101b0733]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2585d2a2]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:57a2a2a6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:90bed341]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:68499a3d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9b9eb0c1]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6d6d5a33]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5c85939a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:46e5af66]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:496bdc05]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6eedf877]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3ebb40ea]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0da54c6b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3fa8eeeb]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:066c4b59]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6c8111b9]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6e314453]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:28228bbc]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4cd8104f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3c660350]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:041ed7dc]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:74e07f22]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1f431059]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:46d6c9ef]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3672afa2]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9c7940e9]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6021cbb5]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3b273279]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3bb90c91]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1f4ed58b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8c500948]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8a1867e4]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2b2bfccd]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:085213f8]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2e06932f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:733d011d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9e34a8b9]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8f66d0e5]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2aed37a1]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5bc61391]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2a03c4fc]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8618e03e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5e352328]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:204dec44]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:97493724]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:17aa802e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:906c637e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5693c099]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6bd1f777]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7fccaf56]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:489bd583]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5098ec3b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:853cdf55]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:90664767]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:953b8f18]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:47c563d7]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:82741542]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:921aad0b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:472464d4]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:19d672be]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:50e64eff]
%   data: {"dataType":"text","outputData":{"text":"--------------------\nRobot: (4 bodies)\n\n Idx    Body Name    Joint Name    Joint Type    Parent Name(Idx)   Children Name(s)\n ---    ---------    ----------    ----------    ----------------   ----------------\n   1        link1        joint1      revolute             base(0)   link2(2)  \n   2        link2        joint2      revolute            link1(1)   link3(3)  \n   3        link3        joint3      revolute            link2(2)   link4(4)  \n   4        link4        joint4      revolute            link3(3)   \n--------------------\n","truncated":false}}
%---
%[output:36738764]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7b951aee]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7271fc34]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2e2e3ea2]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8b7fb3a5]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:78825d5e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4d0a0de7]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4c4b68de]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:97a4b742]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:35bf34ad]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:68d7c888]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3b8f724d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:832cd5d1]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:71ddd7f8]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:222ecb2f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:24f0fea7]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:80491036]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:491d6be6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:23c1e855]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:926fea8c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3e5ab6a9]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5c2b499d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:805c2770]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:65608651]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:986ad192]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:12d17120]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:981844e9]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5a3d508a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:63583468]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0fb03910]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1ae3c5a3]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:64804d7a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5f2e5027]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2223bcd9]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7032a66f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2f12a9f7]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:597bee82]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:22b1400d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:40bc9e8b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5a5f39e6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:027e4fc6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:24701a23]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8822b499]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0be2b954]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:45c00898]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:48d35ca1]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:47b5288b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:414f972d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:28cd6b82]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:370fbbd6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:23473382]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:40aef911]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2c0601da]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:91e40e51]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5da1b0e8]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:34573ed2]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1a184281]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0c5584fb]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:74c2ec2b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:018e2f8a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:355f1e6e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6785b180]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3ff71ff1]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5df68c77]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3e26defc]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:42fb4211]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8bcc2cbd]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:38a55a7a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1c88b3a2]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4486f182]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:09937f66]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8035e616]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6ebad528]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:53e24d6f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:467e6a6e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4e75c452]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5147f3c5]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7145d6a7]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:36924aad]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:09ce2170]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2d882e73]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7c8c2477]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:21e02962]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0b9c46f9]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:270875a0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8c7b415d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:673f91c6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3aa67e7e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6a26c31f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:047302c4]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3651757d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2fd94782]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9aee0727]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:866d64e7]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:79007169]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3bc98fa0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:506e9577]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:683338ea]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:986f4156]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:591a2a0f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:25eb2987]
%   data: {"dataType":"text","outputData":{"text":"--------------------\nRobot: (4 bodies)\n\n Idx    Body Name    Joint Name    Joint Type    Parent Name(Idx)   Children Name(s)\n ---    ---------    ----------    ----------    ----------------   ----------------\n   1        link1        joint1      revolute             base(0)   link2(2)  \n   2        link2        joint2      revolute            link1(1)   link3(3)  \n   3        link3        joint3      revolute            link2(2)   link4(4)  \n   4        link4        joint4      revolute            link3(3)   \n--------------------\n","truncated":false}}
%---
%[output:712c4ea0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3c86b27c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:87083bd3]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3d03ddb8]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9572a411]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:50a7c073]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7c16d1af]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:76f35c5b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:59c6ffae]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:167b6440]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:68b152c1]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9d4e7ab7]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:25ea5cc8]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:46455e13]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:36225231]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:24faa382]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6bd9d2c8]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:072dd645]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:56b335c8]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1451f31b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6fbc1988]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:60b3415d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5d27b3f2]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:984f33ab]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:544ce237]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2fd72dc4]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7b831f70]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8956f6d5]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:763148b4]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:719240bf]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:87bb3278]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:402a89e6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6ed34da6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3b8b77ad]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:639415ac]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:258a9a10]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6c95da56]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7fcb07ab]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:547b3ead]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:795999dd]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:834cc6ee]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8653d882]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4c97ecea]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5391e9b0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2faa6255]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0ea7a4af]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6de8a45f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:34ec755b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:31b3859c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4b5ed953]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2bd05ac9]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:83858fce]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9be892ed]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:717ab206]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:026c2f62]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7d368bcc]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5382ea37]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:43a450bb]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7cb710eb]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6e8052f3]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:32b0dcf0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8f492cc5]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0907e3a3]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1eef8d8f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6efb82f5]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:278804c9]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4f2a16cb]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2fea4f76]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6d9d4819]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:913c133c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0ea7c9c6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:254df142]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:33e21062]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:43ce4daa]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4728c101]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3682ded8]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4d2c0b66]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4e2b82a5]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:216f571d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6e3f58ed]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1b12a300]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:19fe0694]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8cf5404a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:86b07d85]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:336b4c70]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9f55b8fd]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8869b43d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:92fbf951]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:097fa215]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:02f3c7db]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7f9fc1dd]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:44fe1de9]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6261f7a0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0ac06073]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:95a0ce1b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7a008307]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:413d7367]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4d8b18cd]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9421bcd4]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:479dad90]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7184803a]
%   data: {"dataType":"text","outputData":{"text":"--------------------\nRobot: (4 bodies)\n\n Idx    Body Name    Joint Name    Joint Type    Parent Name(Idx)   Children Name(s)\n ---    ---------    ----------    ----------    ----------------   ----------------\n   1        link1        joint1      revolute             base(0)   link2(2)  \n   2        link2        joint2      revolute            link1(1)   link3(3)  \n   3        link3        joint3      revolute            link2(2)   link4(4)  \n   4        link4        joint4      revolute            link3(3)   \n--------------------\n","truncated":false}}
%---
%[output:4ada6a4d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:783dd575]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:10815cdd]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:71ef2b32]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:35decdf0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4037e6f0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:78a390e8]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8d6e326d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:482a5533]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9f31f500]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8350302e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8191a539]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4ea64dc6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1caefc4b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4d0607ae]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:28902f9e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:288b3e13]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5e51b919]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:163c98af]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:38cb9e1b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:81cac3e4]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9569bfae]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1bb951bf]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:375af20d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1a044e3a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:683a46a1]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:26a2009f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4886aaff]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:59162c11]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:68532df0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:99b877ed]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:761e70b4]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:927aa85e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7bdd1a88]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5c96c7ea]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9efeeecf]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1889083c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4d2035ee]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:25d59388]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1834c7ab]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:344f0533]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:46552c9c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:73dd03e0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5ae8bb12]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:45c3e0b3]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:565ddfb2]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6cdacbe3]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:698d541a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4ef7dccc]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1cbff8a5]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3cf7b960]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:71218cb0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:902bbda7]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3123c8a2]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9dae6173]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:985ac666]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7f12b045]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4760906d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:189aba86]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:45f05001]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9f7cfc2e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8671d85d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:836eb06e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:28fd952e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:20bf6899]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1509b444]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:09f058bb]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4a92558b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6757518d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1479d6e8]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5201e7de]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2c7a7912]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:92137c8e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:893a0e47]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:13a51b29]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:17b16ecb]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:961bb7f6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:17ceb9a0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:77f75f4b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0ae0e47f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:071894dc]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9f5446f4]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:323c0278]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:81f5d579]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:549a4b6b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:405298ea]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:42313766]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:76002630]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:35070a34]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3863b032]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6cb24a03]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4b83099c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2002a054]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:460ddca9]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2a4683b5]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:92af0465]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7f33e94d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:00d929fe]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:987a1e5d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7a05d1ab]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:56970a22]
%   data: {"dataType":"text","outputData":{"text":"--------------------\nRobot: (4 bodies)\n\n Idx    Body Name    Joint Name    Joint Type    Parent Name(Idx)   Children Name(s)\n ---    ---------    ----------    ----------    ----------------   ----------------\n   1        link1        joint1      revolute             base(0)   link2(2)  \n   2        link2        joint2      revolute            link1(1)   link3(3)  \n   3        link3        joint3      revolute            link2(2)   link4(4)  \n   4        link4        joint4      revolute            link3(3)   \n--------------------\n","truncated":false}}
%---
%[output:6ff27c96]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:43cd7971]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7f740579]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:68561ec1]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:471c61ef]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:309dc404]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:66297509]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:11f7a08a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:53586b0b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4a632fd0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3560f6fe]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:466f5f14]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7817b20e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2ea18f6d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:33e1252d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:67b5031d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6f2d44e0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:30cae853]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9f1fb520]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:906c2c72]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9d1008a4]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:90b4899e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:074b7bda]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:53c52a78]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:91b7b4ac]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:157849e8]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:610f8645]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4e48bccb]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:970d1739]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9897ca3e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5e2e2644]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2f757f7a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:06b2c14a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5d2bf9a6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:83960ba7]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3f60d49f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:71a2e2fd]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9800d7d1]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:68eb1f6d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:23f03cf6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2b549663]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:02c75646]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5eae9f39]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:724db77c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6ff76f29]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5fe2a7c6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6960f497]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:176be319]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7a4159ac]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:179aff53]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5d2c3ada]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:23dea989]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0cd5c31c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:115cca8d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5814c5fd]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:787e5d2b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:519ad10e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3530ca9b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:73b4c59c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:49ccf3e2]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9301b664]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:95a22f52]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4b54e8df]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9969b3b4]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6d3961a7]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5a001368]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:21a7e1d4]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5be55dc8]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7771dac7]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:07002513]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4977646d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:88de64b3]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:377e3f21]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2b9cb781]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6668d440]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:205dc45f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:34673ccb]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2998c2e9]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3b2f714e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:600ef699]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:501e18ca]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:878e9faf]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8822df20]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1f6b9cfa]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0d769b68]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:979c9e7b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:59d94e01]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:59b89d30]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:84f61951]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4e09b430]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4a081a98]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:960948b2]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8ab9b1f2]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1052a1ed]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:05dfb809]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:433db920]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:04ce6ba9]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4ffdbece]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:66ba0994]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8627e72b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2472061c]
%   data: {"dataType":"text","outputData":{"text":"--------------------\nRobot: (4 bodies)\n\n Idx    Body Name    Joint Name    Joint Type    Parent Name(Idx)   Children Name(s)\n ---    ---------    ----------    ----------    ----------------   ----------------\n   1        link1        joint1      revolute             base(0)   link2(2)  \n   2        link2        joint2      revolute            link1(1)   link3(3)  \n   3        link3        joint3      revolute            link2(2)   link4(4)  \n   4        link4        joint4      revolute            link3(3)   \n--------------------\n","truncated":false}}
%---
%[output:9e8867da]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:185a7833]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:20a02812]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:863e5b54]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:770b5385]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4a8d0a75]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:91c9ae5f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:03ac9bdf]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:11d5ad8c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6f2f4a13]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:97d449fd]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6b80f0ba]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1c7fd3eb]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:742ea4a2]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2ac47630]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:13809a39]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:72d8b240]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:47b6fd3f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1a806885]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:38be4d0d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:49d09cbf]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3c62573b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:03620ffb]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:115f148f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9d32477d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:09f8fa4e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7c59034e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:44565e58]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9c0fd06d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5d19101f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:327baba1]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1d874fc0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4aa4d765]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9f80adc1]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9eab8c4a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7365b63a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:41078543]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:374cf5ce]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:53758233]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4bddca68]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:73abb451]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8feca7b6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:97dbfecc]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:97ada13b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1e3ccd4e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2ac27e94]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4b0fc46e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:00dd1a49]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3db97374]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4c4ac35f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5862122e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:63a542d4]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9345cd27]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:645243d9]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:847f62b3]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5a14c495]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:30b46f26]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:47b1e694]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:713e796d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7be612fd]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:7f12eda3]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:367de070]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:43a0396d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:39ac0fcb]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6269235e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:65110a05]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1574d15c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:3c2b8043]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:43f5144c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:83acd050]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2d2d4933]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:99184f82]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:15b52179]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:4abe06d9]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:69d61557]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8ec2759e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:1637bcd6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9ec03362]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:2b44aa03]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:20060263]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8343756d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8abbcdd9]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:50c563b6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:550aa568]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:53e686d3]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:304bbd15]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:00487bcb]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5f4ea63d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8563c579]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:673ca518]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:9fea9564]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:57745a8a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:5f5cc2b4]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:6a2c3927]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:48b56181]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:0c972d00]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:8cb034b6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:703294b4]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:28bdc972]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:67460edd]
%   data: {"dataType":"warning","outputData":{"text":"Warning: By default, rigidBodyTree\/checkCollision skips self-collision checks between parent and child rigid bodies starting R2022b. In previous releases, self-collision checks were skipped between rigid bodies at adjacent indices. Specify the \"SkippedSelfCollisions\" name-value argument to avoid this warning."}}
%---
%[output:048b7d4c]
%   data: {"dataType":"text","outputData":{"text":"    <strong>PointID<\/strong>     <strong>Status<\/strong>      <strong>Target_X<\/strong>    <strong>Target_Y<\/strong>    <strong>Target_Z<\/strong>    <strong>Euclidean_Error_cm<\/strong>\n    <strong>_______<\/strong>    <strong>_________<\/strong>    <strong>________<\/strong>    <strong>________<\/strong>    <strong>________<\/strong>    <strong>__________________<\/strong>\n\n       1       \"SUCCESS\"     15.167     -6.9626       10.71           6.8221      \n       2       \"SUCCESS\"     15.403      4.1088      5.0754           18.091      \n       3       \"SUCCESS\"     15.342     0.21267      10.778           6.6855      \n       4       \"SUCCESS\"     11.529        2.62      9.7456           8.7508      \n       5       \"SUCCESS\"     14.904     -8.2264      8.7628           10.716      \n\n","truncated":false}}
%---
%[output:0adc5e09]
%   data: {"dataType":"text","outputData":{"text":"Mean Absolute Digital Error: 10.2132 cm\n","truncated":false}}
%---
%[output:21a4b44d]
%   data: {"dataType":"text","outputData":{"text":"\nWARNING: Your digital error is high. Check your Z-offsets or DH parameters.\n","truncated":false}}
%---
%[output:9bce8b1d]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAI0AAAGECAIAAACERDSOAAAAB3RJTUUH6gQaESsZvtAfkQAAG\/NJREFUeJztnX9sG9d9wL8kTyElnWOlHmVTP+DUztqq4mZMLhslUYEqqAo1ae02m6OmqQVoWtJBrYA5NZxGwQCvqJU18Kyu7FwkHuvObZYyWdbEWQa33sIiUVs5mtUaISEHSVSrpkxFdGwpOlpiRN7tj6\/1fD7+Fn9IX+b7gWGQFO\/u3fvc973v3b3HM0UiEWDWPObVLgCTFeyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBtc8eTweZwI+ny\/Vkoqi9PT09PT0KIqSTwkCgUBra6vH48G3Bw4cCAQCADAwMNDZ2RkOhzMui0XN+GWxQv0W028lmzJkJBwOd3Z2iioVe5oTknjV29vb29sbDod3797tcDjcbrcsy2mWlGX56NGjK9ikgebm5pGREXzt8XhefPHFL37xi9ks6PP5+vv79+zZ09vbCwADAwNf+MIXPB5Pc3Nz9lsfHBxc8V+zIRAI9Pb2PvDAA1hILDMA4NvsSdfu4YEwMDCAG3A6nfrXJ06cEPGEn3z\/+9\/Ho9twyOBfMTQHBgb0rzs7O4eHh\/Ho9vl8Q0NDiqJ0dXWJON63b5\/T6WxtbcUgEyiKcuzYMZfL1dXVhZ8MDg6OjIwISbghp9OZPuL1EZO4iP6v+rAQO4hfOHDgQKpCHjx4sKmpSRSyvb3d7\/cLSaIN0xdSfCg27fP50nmy2+0tLS1jY2PhcHhiYgIAQqGQoigTExMNDQ1NTU2G709PT4+MjOzYsePZZ5\/VtxVOp7OhoeHkyZOKooRCIQCYmJjA1y0tLTfddJPYhz179siy7PV629vbASAYDHZ3d\/t8vpqamqeeekq\/rcnJyfHx8ba2tqRBPzAwMDY25vP5fD5fKBTKJix8Pt\/x48fdbrfX6x0fH\/d6vfq\/YjPT0tLi9\/vdbvfQ0JA4koLB4O23376CQuJx6Xa79YX0eDxHjhzxer0jIyMOh2Pfvn3oL0Me0dHRMTs7OzMz8\/bbbwPA+Pj4+Pj48PBwS0vLhg0bDF\/eunUr\/o+LiM\/RdygUunjxInoaHh7GVXV0dKTZuizLtbW1lZWVDocDD5H0pUXC4fDY2FhLS4vdbrfb7bt27Xr55ZcNR3oq+vv7Z2ZmRkZGDO2S3+8PBoNYWpfL5XK5jh07huXJspD6cMToOXnyZENDg9PpFPUzMzMzPDzc1NS0efNmWZa7u7tHR0dHR0cho6fa2loAeP7558fGxu67776amprf\/va34+PjqMTAli1bUq1n69at4+Pjx48fn52dffDBB0Oh0C9\/+Uux\/lTU1NSk\/0JSZmZmZmdnk5YwDVj7ANDf35+YQE1MTKCPnApZW1tbU1ODh7jdbj9x4sTIyAhu5cqVK6FQKBgMtre3O53O48ePh0Khd999NxQKORyOxPjL4Gnz5s1NTU2vvPKKpmmf\/exnHQ7Hk08+CQCtra3ZVgCA+P4rr7zS1NR02223zc7OPv3003jg5LQeQ8GGh4cTj1997WQPpkV+v3\/Pnj0A8N3vflffdG\/ZskVRFH0jkQ1pgqyqqsrhcDQ0NPh8Pr\/f7\/f7T5w40djYmOr7GTzJstzW1jY1NWUymW6++ea2tjYAWEH9YrWePXvW4XA0NTVh35aq4c4G0SyIjmRgYAB7cn23Gg6Hn3322TvvvDNjEujxeHDxrq4ul8vlcDgqKyvFX0UXCwDYFnV3d2csvCzLe\/fuHR8fFx3k4OAgtmMA0NHREQwG\/X6\/yNewtsfHxycnJ0WihPEnpdzIMq2trbIsY3OPLRsGZk6nTViC0dHRjo4OWZYdDgckaydbW1uPHDnS1dWFB3V62tvbvV5vb2\/v0NAQbkIk5YODgwMDA5iM7NixI5s8ore39+2338bEDFel12C323\/yk5\/s3r3b6XQCgNvtxpVnpLm5+cUXXxQLAkBDQ8Pjjz8uyzLmTZimu1wuzKWxX8RiuFwucXZkikQi2WyPWV34uhEN2BMN2BMNMucR5IjFYvi\/JEmSVCY7SGw30AG+wNfRaBQA4vE4\/h+LxSwWCwCYTCYA0DRNVVVJkjRNs1gskiRZLBZN04RCKi7XVhFFKIgXWPUmk0m8MJvNsVhsYWGhurq6oqICa9lqtYp6Fx\/iGpaWlgzrjMVii4uLsVgsEolUVlaazWZcEI1aLBY0vaZcli4vT3SA9Y4yMCBQA74wmUwVFRVwfX3hJwAQCoUaGxttNtuKC7O4uBgMBjdt2iRJUlKXmqZpmgbLQQkAGIso0qCw2C4Ls+qkDkDXHAkNqqouLCxUVVVVVlaaTCZJkrCuhQNpmfSby7NScBNWq1WW5VSrEvsCKeJyaWnJEJRoEQDMZnOiwnxcZl4slQN9NGBnIDoGSZIwGvDSi95BLBbLMxRKhqjcNN\/BuzN2u91msyW6RJEYlxiR+FrfX2bpUpqamorFYhs3bsRVLy4uQjIHhjioqKjAbkDvIM1mBPmHwpoC9zdNUELauNQ0DT9ZWlpSFKW6utpsNmNjq893JEmSXnjhhVdffXXv3r0Oh0PEgeiKRUAsLCxcvHiRRBz8+vTwD3\/1Lzc8c+vWCue7j7zQ9af33\/qR3K7uF5Z84hJfLC0tSXV1da+\/\/rrJZLrllluy2d7a58zc75p3ftR8h\/Kjnz4wd3B+4331q+spGzBmbDZbqjAwX71sTsSBHnEu9UGgPK8baXA1pU4DLc2r4ynPOsoY\/RpoABk8ZbmqNUJ5xhMGVAm2U7KgLLWn0hy\/mqZmGU8FoQQ7VZ7xJC75lA1XPU1NTa1uOQqNppUwnkqAub6+frXLUHjUsgsoGtlOUvBcPSmqppo0dcWLr0EIe0pDXI2DZsrY8lFJyoGup\/RVHI\/HIUM4FYaSBeXqeCr27qmqir1T2XRRq+CpBK1NXI2btGwvSeRJaRpPqu1eemJqzKxayieahKdgMLi65Sgs8eV2r2wox3jSIK7G8T532SABQF1d3WoXYyWkvgaqqfG4Sc1wSYzWfY1yjKer509qGfZPJSb\/YznD+ROe5+a3kmwoWVCWbzypmT0RokzPn+JxiJtK0+zx+dMK0QDi8bgWX+1yFBTy50+xZRYWFvDt7OXZeFVcW+44FEUJh8NiOOLaH3+YFErxpFdy6dIlANDP5UYTNptNkqR4LK7Frl42wsHS+M3Z2Vnx5cuXL1utVpvNVllZuRZmZKRnjRYuMUpwQDWCVS9JUk1NDQ7d1UeJLMvxeVWNX+2ebDYbTq8XA7PxKvClS5fm5uZuuOEGvTyxKhwIvHaCTwKA1b2lq1eiH+OOiPHVdrsd32Ld4fhe8dtIBlQ1rsaN12ExYsSvDeBWHA6H\/oBYXFxUFCUWixnkiZH0hmOiZJQunsRA+CtXrsR0XCtKCiUrQFW1bK7w6ecuGLZlkCdaTv2CNpvt0qVLFotlcXGx2PKK4kkfFhgleiXvvvuuJEk33ngj9g2Qn5KkaKqqqnmNZEkjT8x8isVic3NzsDyhSN9BQqFzlsKckKdSIg5Y0V0DgCzLsVgM+4wioamaVoTrsKnSDbvdjt0e7ntizqLv9laWs+SwgL4Q2ShJVaD5+flcS5kbGqgYT1CKO4ViCpT+Q0POsri4iNpSydN3ycm3kvTTxCiZm5sLh8PxeFzMWMYAT69ktVBVTVXxOuzqXIs15CyCVDlLNBrF6q2urk6as0iHDx+emppSVfXy5cu4FkO6BQA2mw0bq02bNqWfO1dK0lwD1VRVyzQubFVI1e3Nz89Ho1FZlnGGYWLOIrlcrsOHD9vtdvwbKkmMEgxh0UmucVT12vlTKmJrZgYqNks33nhjTU2NUGjIWSSXy7Vz587p6enGxsbVLW5OpK1iTbva7mVQtUY8JcXQlZCc\/5QeTQO1OPleIiW7\/1Sm8zXyPn9aa6yCpxK0Nuq1xLzolKbxLM94UlVtbeZ7K6YcPWn4i0Rl1OqRnqeW\/vxJjWcYb1SyFKAglGM8LfdPq12KQkI1Lxc\/8JaULPPy9CtZU5hhte8TFhyLzbz8w3kAGkiVluJtq5zPn0pxFKuglWUeUW4s33MvgavSNJ7l6Em7+rs55TTjvQw9Xf1J1zJKykF4IjrOMlV1X53zWaKZn6WgDOMJINvfoVrL9zUMUPWUroqvl1S8iCpl42mGVZpPWOyd1MrrF46oxlN6SimpbO9rFH3HrmYQZRRNa3YeQE6IUWw4hO3y7GXQMOXTAECZn798+TJQnlQDFD2hFZzVVFlZqWmaGMh2deSbJBl+NEc\/xhG\/YzabL1++rB\/fs8Yh4EkMKcRwEU8siMfjf\/jDH6qqqqqqqmRZxnG4NputulrWn+jKstzY2Ihj5HBEYyQSmZ+flyTp4sWLs7OzYkzdKk7HyMg1T8FgsKGhYRWLghim2ehjBQBsNpvdbsea\/fCHP4xDoBeXEXGj6U5yFUU5f\/48rgTHH27YsEEMdhdDU8WIYtCNhlw75lb5dwmysQKpJ3Tg\/CdcEB9REYlEoovRa0mEBpFIJBqNVldX68ePijXoV2sYVGwwh4715gzzgopKST2Jipifn3\/ttdcAQJIkq9VqtVqrq6v1Q3HTHMLp1W7YsGH9+vXLeQQAwPr16zdu3Ij1rh+GL8bHXxvDnTCoONEcPr4tGo3ic2KsVit+p9hJbBHnExrSMP3R96EPfSgajeK+LS0t4d5ijUvLT83SH7M5BZzNZtN0ebnNZhPTDvWrwmHcIrmA5WH7wlxM9xwl3DR+TTxDzGKxqKoqHi+e1H2hKNhRoLcC18+oFcdpqljRN1zvvPNOLBaLRqP4oDucHpJTwAGkPHlKGjG4aQwXfLqcPlwAoKqqqqamJtWm07svlLnc5j8Z3iamYXD99M2MLRgkBJzVarVYLDabLR6PJwYcRps+4JKiZXEd1rB1LH91dTXGin7TAKAoiiRJsVjMZrMtLCxkbC0N09bSRK1YJH1Xl4MnVVXfeOMNfFwa7oB+LpRIw1K11AUJOMOEr+RNzXKyZzCVpgmVJClNuMDy3LrErSdNC6Vk09ZE+XHO08WLF\/EZaNFo1Gq1KoqSfsKSlHTkXtJdwmfu4RcsFgvugKippHuYZ8BdK+X1e27o3vUHrCRJi4sLcC2cNHwEVqr8PuPW8fv6ifWJyYX+y4bpe4bPKyoqsDbQFrauAGC1Ws1m8\/nz5\/U1eV3Ujo6OHj58GJbP8GMJ89RS9QqxWEwcZYqiiAPEZDLhr7xji5F9wGVP0qZmfn5+2dnc1dFGmgYAESUyNzdXXV190003ZXlYrGDricdNbPmpgfoKEbWBT6\/Tz3aC5UlmSaNWEsneXXfdtWnTpm3btm3atOmOO+5oa2vLmBzj0SE+xBQuGo1imbCzwWeEFTwFStq7YBXs3P6l5i3O8OxMfFH9kwr7x2\/+OBYGfz0n6dGaJ+L4wzqRJElRlHg8LvIRw1GLHYf+eNV3XWIHg8HgW2+9de7cudOnT199fu7U1NQLL7wAAKOjo6Ojo\/jVhoaGW2+9Ff\/\/xCc+kVgpoEuIDQGnj7ak\/VCudZS+dxFNTarV6nuIxPIbe7isiyTyBfyNCdC1H4kVIgqgzxr0h87c3Nz09PTU1NTIyEgwGDx16hQs3x1M8pxj7LEStYlo2759+6c\/\/emc2pCk1ZReGy6SdGJ9xin12ZRHn0wnFinpFaM0e5HxQElaBgyXc+fOnTlzZnp6+syZM7AsxuVyuVyu+vp6fDBh5udRT01NXbhwAW2lirbW1twe1Ji4w9ip4jNgAQAfG5xPLeRK0gYgFovhw8QNbdeKixQMBoPBIEbMqVOncPiQEINWUIyBnJ8bniractWmbzTwuhw26LHrz21lWa6oqCjZ\/G0RxIuLi3hhEFXhY4nxVASvzWdZKmy+UE+iGH3EpCff57tnr03fjqVvNFI1L5DiJDEfsiyV4SwqaanWrVsnSZLwgRGDX6urq6uvr6+rq8tejIF8PRlIo2379u01NTXbtm274447ICHvSIPoS\/LXlipL1IvJMnZFqd56663Tp0+fOXPm4sWLp0+fxr\/mL8ZAgT0Z0Gt7\/vnnL1y4gJ\/n2bclagPdubBeWxox+uw8p0YVI0afkiFopVBiDJgikUg4HN69ezc2nXv27Ont7QUAj8czNDQEAG63u729Pf8tYXjt3Lkz\/75Nj0EbiolGo6qqWq3WxcVFcX1rxfl3UjGGiBkdHU2VAqwYvRdTJBIZGBjYunVrb29vIBDo7e197LHHamtr9+\/ff\/jwYb\/ff+zYMbfbnfhTPflTkJTEEDH6E0y8CIkpSdJoSwrWS3ox6Cbf\/c+E3st17Z6iKP39\/d3d3RMTE8PDw263e2Fhoa+vb\/\/+\/c3NzcUuVv4pSdKzS3GeZPjRIAyvdevWTU9PQwoxGB8lE5MKRVGua5cnJycVRXE6nRMTEw6HQ5ZlvPc6MzNTAk9YEX19ffjWoO25557Dzw0pSfrLqfqzfbvdjsF39uxZAHjppZcSO3+Xy9XX11fwFixPJicnr3kKh8P79+\/fv38\/3iFddVJpE5e4kGwayVRNGbIGxehBL5LP52tvbw8EAtghCUmhUAgbinXr1tXW1q5sGwVMRoS2qakpTKjq6uqSRpvQZrhQBssR853vfAfXgMp37txZqDbN5\/P19\/eDLh3LlYGBgePHj8NyjQkvpkgkEggEnnjiicHBQZEsiD\/nk0cUZCVZkqpvK2qubCAcDmNfDgCGg35l6L1IiqIcPHhwdHRUNBposrOzs729XZZlj8ezsvodGRnBG1dOp1NRlMnJyeJ1ckkbyRL3\/H6\/f35+vra2trKyUpZlv9+fTxNi8CLJsnz06NHE7\/X29q4scvWUPhlBVis3czgc+AvSADAxMZGPJ4OX8pxXU34U19MKkpFwONzZ2el0Op1Op8fjwQ89Hg9+4vP5Cl5IRVF6enpw\/QMDA4Zi9PT06E+80hMKhcQN7i1bthSwkEX01NraqijKwsKC3++XZXnz5s3ZLDU0NLRr1y6\/3+\/1eo8cOeLz+QKBwIkTJ3w+n9vtPnbsWPa1liVer9fhcPj9fp\/PNzY2hgcHFmNkZAQARFaSHqfTuW7dupmZGXEaWsBCFvGmTnNz8wqSkcHBQXyxefPmpqYmKH4+Irphu93e0tICAOFweGxsrKOjQ5bltra2kydPZtPT2O32vr6+rq4uAHC73YU9DS3uzbd8kpHSXxwJh8Nvvvnm\/fffD9c31Nh6Z3Octbe3+\/3+YpRtjeYRpb84oijKvn37+vr6SpaU5sRa9BQIBPr6+g4fPiyqrCAXR9IQDod7enr27t0r2jcMXHyN0VzwjebEmvOEJ+FHjx4VkbSyfCR7wuHwvn379IcFdlQTExOKogwPD3d0dBR2iyuguPdzcwVvrOjzK7w4gtcJMR8peLskLqkheGlO3KPbsWOHSG1WkRw8YdF37dolUoPET5iM+Hy+Rx55RBxw4t5s+pQyt3gKBAIPPfTQoUOHcBt4VrgWDjdaeDwevA0LAP39\/W1tbRkP9Nz6p+bm5nvvvffgwYOKogQCgTfffHPPnj0rL+8HFTzH8nq9Xq9XvE1Pzv0TtnXf+MY3nnvuue7u7oIMcfkAgs0dAGTZ4+ac79nt9ocffvhb3\/oWAKzZe6BrH7za0tTUlGX6upK8HO+8dXd3r\/pZBV28Xq+iKIqiYNOXEQK\/x1J+BAKBZ5555tChQwDw0EMPtba2Zmz61tx5btkTDoe\/+c1v3nvvvc3Nzfq8LP1S7KnU7Nu3z+FwiBwPX2Q8t1lb1yOYVHA80YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90SBbT7\/4b4ujukr\/7weHKopaMkaPKRKJ5LrMDw5V\/PhJ6b9+tbhpk1aMMjGJSLkucOZ35qNPSD\/6WZQllZLc+qfpaVPvfdaer8W2\/YVapAIxScmh3VMU2P1XtoZG1X3k\/aKWiUkkc7v3m1fNf\/c1KwBs266eP2f64Y+Xil8qxkhmT+cnzecnTRrAOyHL8Zc5d1gdcuif3n8fOttsIi\/vf+CG4hWLMZBtvmcC+N4T73d9NVbU0jCp4OsRNGBPNGBPNLA8+uij+vcLR3743td6pFs+os7MLL3yq7m\/\/WvHqX+TFuZevevp31UfmYXzr54fMZkAwFRjuxEAun7+N\/9x9viuph0A8N7785956i\/Pzf6x\/ea21diXcibJee7sl78UO\/1\/YDIDaBqASQMwmTY\/YEzHT3zZ+6nG27zjP3\/UN+i7\/\/nG9fW\/Dp76+omHX9z1VOP6+lKV\/4NCknav6sGvgwk0TQMwmcAEJpO5viHV8rfXfRIAfnPhNQD4xdu+j264hSUVgyR5+Q13fkZy\/nnM77d+7m7bV7rNDQ2W+oYIwORc8I\/vnZ+cC07OBf\/4XvBTjbcBQOP6+u2ObYGZN6AJ3rj01j0fu7vku\/CBIPn5k\/Vzn4\/5\/WC1VtzaKj7cvL5h8\/qGTzUav3zPx+7+wagnED57YX4aw4spOEnavZj\/9cWf\/bv8Dwfef\/lk9H\/\/J+Mqbq\/75Pz7yr\/+\/qd16zZxo1ckjPGkKYry+AHbl79iu++rmjJ\/5bFvS84\/s2zcmGYVjevrP7rhlid\/f+xHn\/\/nYhb1A40xnpRv\/7124cINO+4BANt9u011dVf+6R8zruWej93tqN7IjV7xMMbTuseHxGuTLNcc+1mWK9ru2MaNXvEozPWI\/zz7Emd6RSVfT78Ontr4vSYA6Gr6UiHKwyRnJeONmNLD12FpwJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5o8P96Pr\/hNZvwcwAAAABJRU5ErkJggg==","height":1541,"width":560}}
%---
%[output:4e54c4e9]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAI0AAAGECAIAAACERDSOAAAAB3RJTUUH6gQaESsZvtAfkQAAHBtJREFUeJztnX9QHNd9wL97t\/gOWCwc9ZAOwciR3DqEazVFuRjbZCZkQobYiZS4lYnjiBlK7XRImKkcjZzg6YyaiXDjUUUSUjK2elGqxLXPrhtbrjtK1PoyNkmQqUg0vhvksU1EdOgwJwswi+DM3W7\/+Iqn1d5v7gd88fczGs1x3O2+fZ\/3fe+7u++x0vz8PDBrHstqF4DJCPZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEA\/ZEg2uePB6PKw6fz5fsm6qqdnR0dHR0qKqaSwkCgUBjY6PH48EfDx06FAgEAKCnp6e1tTUcDqf9LhY17YfFBo17TL2XTMqQlnA43NraKqpUHGlWyOJVZ2dnZ2dnOBzeu3ev0+ns7+9XFCXFNxVFOXbs2Ap2aaK+vn5oaAhfezyeF1988Qtf+EImX\/T5fN3d3fv27evs7ASAnp6ez3\/+8x6Pp76+PvO99\/b2rvi3mRAIBDo7Ox944AEsJJYZAPDHzEnV72FD6OnpwR24XC7j65MnT4p4wnd+8IMfYOs2NRn8LYZmT0+P8XVra+vg4CC2bp\/P19fXp6pqW1ubiOMDBw64XK7GxkYMMoGqqsePH3e73W1tbfhOb2\/v0NCQkIQ7crlcqSPeGDHxXzH+1hgW4gDxA4cOHUpWyMOHD9fV1YlCNjc3+\/1+IUn0YcZCijfFrn0+XypPDoejoaFhZGQkHA6PjY0BQCgUUlV1bGyspqamrq7O9PnJycmhoaFdu3Y9++yzxr7C5XLV1NScOnVKVdVQKAQAY2Nj+LqhoeGmm24Sx7Bv3z5FUbxeb3NzMwAEg8H29nafz1dZWfnkk08a9zU+Pj46OtrU1JQw6Ht6ekZGRnw+n8\/nC4VCmYSFz+c7ceJEf3+\/1+sdHR31er3G32I309DQ4Pf7+\/v7+\/r6REsKBoN33HHHCgqJ7bK\/v99YSI\/Hc\/ToUa\/XOzQ05HQ6Dxw4gP7S5BEtLS0zMzNTU1Nvv\/02AIyOjo6Ojg4ODjY0NGzcuNH04e3bt+P\/+BXxPvoOhUKXLl1CT4ODg7iplpaWFHtXFKWqqqq0tNTpdGITSV1aJBwOj4yMNDQ0OBwOh8OxZ8+el19+2dTSk9Hd3T01NTU0NGTql\/x+fzAYxNK63W632338+HEsT4aFNIYjRs+pU6dqampcLpeon6mpqcHBwbq6uq1btyqK0t7ePjw8PDw8DGk9VVVVAcDzzz8\/MjJy3333VVZW\/va3vx0dHUUlJrZt25ZsO9u3bx8dHT1x4sTMzMyDDz4YCoV++ctfiu0no7KyMvUHEjI1NTUzM5OwhCnA2geA7u7u+ARqbGwMfWRVyKqqqsrKSmziDofj5MmTQ0NDuJcrV66EQqFgMNjc3OxyuU6cOBEKhd59991QKOR0OuPjL42nrVu31tXVvfLKK7quf+Yzn3E6nU888QQANDY2ZloBAOLzr7zySl1d3e233z4zM\/PUU09hw8lqO6aCDQ4OxrdfY+1kDqZFfr9\/3759APDd737X2HVv27ZNVVVjJ5EJKYKsrKzM6XTW1NT4fD6\/3+\/3+0+ePFlbW5vs82k8KYrS1NQ0MTEhSdLNN9\/c1NQEACuoX6zWc+fOOZ3Ouro6HNuSddyZILoFMZD09PTgSG4cVsPh8LPPPvupT30qbRLo8Xjw621tbW632+l0lpaWit+KIRYAsC9qb29PW3hFUfbv3z86OioGyN7eXuzHAKClpSUYDPr9fpGvYW2Pjo6Oj4+LRAnjT066k2UaGxsVRcHuHns2DMysTpuwBMPDwy0tLYqiOJ1OSNRPNjY2Hj16tK2tDRt1apqbm71eb2dnZ19fH+5CJOW9vb09PT2YjOzatSuTPKKzs\/Ptt9\/GxAw3ZdTgcDh++tOf7t271+VyAUB\/fz9uPC319fUvvvii+CIA1NTUPPbYY4qiYN6Eabrb7cZcGsdFLIbb7RZnR9L8\/Hwm+2NWF75uRAP2RAP2RIP0eQQ5otEo\/i\/LsiyvkwMkdhjoAF\/g60gkAgCxWAz\/j0ajVqsVACRJAgBd1zVNk2VZ13Wr1SrLstVq1XVdKKTicm0VUYSCeIFVL0mSeGGxWKLR6MLCQnl5eUlJCdayzWYT9S7exC0sLS2ZthmNRhcXF6PR6Pz8fGlpqcViwS+iUavViqbXlMvi5eXxDrDeUQYGBGrAF5IklZSUwPX1he8AQCgUqq2ttdvtKy7M4uJiMBjcvHmzLMsJXeq6rus6LAclAGAsokiTwkK7zM+mEzoAQ3ckNGiatrCwUFZWVlpaKkmSLMtY18KBvEzq3eVYKbgLm82mKEqyTYljgSRxubS0ZApKtAgAFoslXmEuLtN\/LZkDYzTgYCAGBlmWMRrw0ovRQTQazTEUioao3BSfwbszDofDbrfHu0SRGJcYkfjaOF5m6FKemJiIRqObNm3CTS8uLkIiB6Y4KCkpwWHA6CDFbgS5h8KaAo83RVBCyrjUdR3fWVpaUlW1vLzcYrFgZ2vMd2RZll944YVXX311\/\/79TqdTxIEYikVALCwsXLp0iUQc\/PrM4L\/4vm9\/5o6bS+ovP\/zCfbfuve3W7K7u55dc4hJfLC0tydXV1a+\/\/rokSbfccksm+1v7nJ0Z+dPPbpPcUz97+vuzR1Tnl2tX11MmYMzY7fZkYWC5etmciAMj4lwqHkmSLJIFpGIWp7Csz+tGFosFh9UUpNC8BlmdMMqxjtJGv0WySFJGTZBKR7Ie40mSJMlikSxQ+I6vaEFZbE\/Fab+SRSrmoRXhoNZjPAFIIEmSBFIRIqpIXG0IExMTq1uO\/GIBCSzrRREAAMhbtmxZ7TLkHwksAOkSPlIQ7vfwXD0hOuhpe7wUX1+D0MhKs0UDTdIkPd3HqCTlQDeeUlexpmt4NbPQFC0oV8dToQ9P1zVdixV0F0VmFQK\/CL1NTIsVJZwAitV5kumgs6KYnorDVU\/BYHB1y5FfYpqm6wDrSNX6jCdNi+nauhIlA0B1dfVqF2MlpLgGGtNiabNyvq+x+miato5iCYDo\/SdIl2XFtJiEiURKW7mnakULyvUZTzE9JmnrKqDW5\/mTFtNA14vT9fH500rRIabHQFtPl8vpnz9Fl1lYWMAfp2emY\/YY6Fc9qaoaDofFdMS1P\/8wIZTiyajk8uXLAGBcy40m7HY7rrwQ1yNwsjR+cmZmRrw5PT1ts9nsdntpaelaWJGRmjVauPgowQnVCFa9LMuVlZU4ddcYJRWKEtM0fTmPsNvtuLxeTMzGq8CXL1+enZ294YYbjPLEpnAi8NoJPhkAVveWrlGJcY47IuZXOxwO\/BHrDuf3ir+NZCKmRePTCIwY8dcGcC9Op9PYIBYXF1VVjUajJnliJr2pTRSN4sWTmAh\/5cqVqIFrRUmiZAVoMU3P4LaGce2CaV8meaLnNH7RbrdfvnzZarUuLi4WWl5BPBnDAqPEqOTdd9+VZfnGG2\/EsQFyUxKPDqBpmhbLKS9PIU+sfIpGo7Ozs7C8oMg4QEK+c5b8nJAnUyIarBiuAUBRlGg0imNGQdAhFo1lEk\/ZkizdcDgcOOzhscfnLMZhb2U5SxZfMBYiEyXJCjQ3N5dtKbNF0zStWNcjxBIo45umnGVxcRG1JZNnHJIT7yXhu\/FRMjs7Gw6HY7GYWLGMAZ5ayWoRi2l6TC\/OFImEmHIWQbKcJRKJYPWWl5cnzFnkgYGBiYkJTdOmp6dxK6Z0CwDsdjt2Vps3b069dq6YpLgGqsViWgwlra2rfMmGvbm5uUgkoigKrjCMz1lkt9s9MDDgcDjwd6gkPkowhMUgucbRYtqyp6RE18wKVOyWbrzxxsrKSqHQlLPIbrd79+7dk5OTtbW1q1vcrEhVxTrENE2LaTltZLUxDSWrMy+s0LdttFgsFivGrUK+\/5QTsQz6PVqs2\/tPmqZBUfK94nSe6zKedOz31lNAUZ1fngJdB03TYrHYGsvJc+KqJ4rr1FKM4bHYcr6XXFXRUoC8sA7jCa7me+tqHQDVvFz8gbcE6KBpeiZ5eaqNrDEssNr3CfOOtdQqkj0dQC6zFm5fRes8VyGeitCKdU1fZ1Ni1+X4pINuOHcqsK3idJ7r0ZNuELVeQmo9egIAHJ7WS1IOwhPReZbJqlvX9TSiqLEO4wkFZTK9fC3f1zBB1VPqKr76F60LHE7F7DwtsErrCQt7kDoU52J50aAaT6nQQS9islecznMVPBXjwISq9ZJLkBlIUyBmseEUtunpad2QRahzc9PT00B5UQ1Q9IRWcFVTaWmprutiItvVmW+yrOvLqbkOsDy7Ef\/Hz1gslunpaeP8njUOAU9iSiGGi3hiQSwW+8Mf\/lBWVlZWVqYoCs7DxXltoF\/r+JQKpba2FufI4YzG+fn5ubk5WZYvXbo0MzMj5tSt4nKMtFzzFAwGa2pqVrEoiGmZjTFWAMButzscDqzZD3\/4wzgFenEZjJjp6Rnjaa46p164cAE3gvMPN27cKCa7i6mpYkYxGGZDrh1zq\/x3CTKxAskXdOD6J\/wiPqJifn4+Eoksn0ABAOA75eXlxvmjYgvGzZomFZvMoWOjOdO6oIJSVE+iIubm5l577TUAkGXZZrPZbLby8nLjVNwUTTi12o0bN27YsOFqMOkAABs2bNi0aRPWu3Eavpgff20Od9yk4nhz+Pi2SCSCz4mx2Wz4mUInsQVcT2hKw4yt70Mf+lAkEsFjW1pawqPFGpeXn5plbLNZBZzdZodr4aTb7Xax7NC4KZzGLZILWJ62L8xFDc9Rwl3jx8QzxKxWq6Zp4vHiCd3ni7y1AqMVuH5FrWinyWLF2HG988470Wg0Eongg+5weUhWAQeQ9BpswojBXWO44NPljOECAGVlZZWVlcl2ndp9vsxlt\/7J9GN8GgbXL99M24NBXMDZbDar1Wq322OxWHzAYbQZAy4xevoLsaa9Y\/nLy8sxVoy7BgBVVWVZjkajdrt9YWEhbW9pWraWImrFV1IPdVl40jTtjTfewMel4QEY10KJNCxZT52XgDMt+Erc1ehX03I9biPJulBZllOECyyvrYvfe8K0UE60bE2UH9c8Xbp0CZ+BFolEbDabqqqpFyzJCWfuJTwkfOYefsBqteIBiJpKeIQ5Bty1Ul5\/5Kbh3dhgZVleXFxYtqTD8iOwkuX3afeOnzcurI9PLowfNi3fM71fUlKCtYG2sHcFAJvNZrFYLly4YKzJ66J2eHh4YGAAls\/wo3Hr1JKNCtFoVLQyVVVFA5EkCZ\/Chz1G5gGXOQm7mrm5uWVnszpeL9cBAObV+dnZ2fLy8ptuuinDZrGCvce3m+jyUwONFSJqA59eZ1ztBMuLzBJG7bXnAdx1112bN2\/esWPH5s2b77zzzqamprTJMbYO8SamcJFIBMuEgw0+IyzvKVDC0QWrYHfDF+u3ucIzU7HF2J+UVH305o9iYfCv5yRsrTki2h\/WiSzLqqrGYjGRj5haLQ4cxvZqHLrEAQaDwbfeeuv8+fNnzpy5+vzciYmJF154AQCGh4eHh4fxozU1Nbfddhv+\/7GPfSy+UsCQEJsCzhhtCcehbOso9egiuppkmzWOEPHlN49wGRdJ5Av4NybA0H\/EV4gogDFrMDad2dnZycnJiYmJoaGhYDB4+vRpWL47mOA5xzhixWsT0bZz585PfvKTWfUhCasptTb8SsKF9WmX1GdSHmMyHV+khFeMUhxF2oaSsAwYLufPnz979uzk5OTZs2dhWYzb7Xa73Vu2bMEHE6Z\/HvXExMTFixfRVrJoa2zM7kGN8QeMg6p4WB0+NjiXWsiWhB1ANBrFh4mb+q4VFykYDAaDQYyY06dP4\/QhIQatoBgTWT83PFm0ZavN2GngdTns0KPXn9sqilJSUlK09dsiiBcXF\/HCIKrCxxLjqQhem8+wVNh9oZ54McaISU2uz3fPXJuxH0vdaSTrXiDJSWIuZFgq01lUwlJVVFTIsix8YMTgx6qrq7ds2VJdXZ25GBO5ejKRQtvOnTsrKyt37Nhx5513QlzekQIxluSuLVmWaBSTYeyKUr311ltnzpw5e\/bspUuXzpw5g7\/NXYyJPHsyYdT2\/PPPX7x4Ed\/PcWyL1waGc2GjthRijNl5Vp0qRowxJUPQSr7EmJDm5+fD4fDevXux69y3b19nZycAeDyevr4+AOjv729ubs59Txheu3fvzn1sM2LShmIikYimaTabbXFxUVzfWnH+nVCMKWKGh4eTpQArxuhFmp+f7+np2b59e2dnZyAQ6OzsfPTRR6uqqg4ePDgwMOD3+48fP97f3x\/\/p3pyJy8piSlijCeYeBESU5KE0ZYQrJfUYtBNrsefDqOX6\/o9VVW7u7vb29vHxsYGBwf7+\/sXFha6uroOHjxYX19f6GLlnpIkPLsU50mmPxqE4VVRUTE5OQlJxGB8FE1MMlRVva5fHh8fV1XV5XKNjY05nU5FUfDe69TUVBE8YUV0dXXhjyZtzz33HL5vSklSX041nu07HA4MvnPnzgHASy+9FD\/4u93urq6uvPdgOTI+Pn7NUzgcPnjw4MGDB\/EO6aqTTJu4xIVk0kkm68qQNSjGCHqRfT5fc3NzIBDAAUlICoVC2FFUVFRUVVWtbB95TEaEtomJCUyoqqurE0ab0Ga6UAbLEfOd73wHt4DKd+\/ena8+zefzdXd3gyEdy5aenp4TJ07Aco0JL9L8\/HwgEHj88cd7e3tFsiB+nUsekZeNZEiysa2gubKJcDiMYzkAmBr9yjB6kVVVPXz48PDwsOg00GRra2tzc7OiKB6PZ2X1OzQ0hDeuXC6Xqqrj4+OFG+QSdpJFHvn9fv\/c3FxVVVVpaamiKH6\/P5cuxORFVhTl2LFj8Z\/r7OxcWeQaKX4ygqxWbuZ0OvEvSAPA2NhYLp5MXtbjupr1SGE9rSAZCYfDra2tLpfL5XJ5PB580+Px4Ds+ny\/vhVRVtaOjA7ff09NjKkZHR4fxxCs1oVBI3ODetm1bHgtZQE+NjY2qqi4sLPj9fkVRtm7dmsm3+vr69uzZ4\/f7vV7v0aNHfT5fIBA4efKkz+fr7+8\/fvx45rWWIV6v1+l0+v1+n883MjKCjQOLMTQ0BAAiK0mNy+WqqKiYmpoSp6F5LGQBb+rU19evIBnp7e3FF1u3bq2rq4PC5yNiGHY4HA0NDQAQDodHRkZaWloURWlqajp16lQmI43D4ejq6mprawOA\/v7+\/J6GFvbmWy7JSPEvjoTD4TfffPP++++H6ztq7L0zaWfNzc1+v78QZVujeUTxL46oqnrgwIGurq6iJaVZsRY9BQKBrq6ugYEBUWV5uTiSgnA43NHRsX\/\/ftG\/YeDia4zmvO80K9acJzwJP3bsmIikleUjmRMOhw8cOGBsFjhQjY2Nqao6ODjY0tKS3z2ugMLez80WvLFizK\/w4gheJ8R8JO\/9krikhuClOXGPbteuXSK1WUWy8IRF37Nnj0gN4t9h0uLz+b71rW+JBifuzaZOKbOLp0Ag8NBDDx05cgT3gWeFa6G50cLj8eBtWADo7u5uampK29CzG5\/q6+vvvffew4cPq6oaCATefPPNffv2rby8H1TwHMvr9Xq9XvFjarIen7Cv+\/rXv\/7cc8+1t7fnZYrLBxDs7gAgwxE363zP4XA8\/PDD3\/zmNwFgzd4DXfvg1Za6uroM09eV5OV45629vX3Vzyro4vV6VVVVVRW7vrQQ+Hss649AIPDMM88cOXIEAB566KHGxsa0Xd+aO89d94TD4W984xv33ntvfX29MS9L\/S32VGwOHDjgdDpFjocv0p7brK3rEUwyOJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5okKmnX\/y31VleZvz3wyMlBS0ZY0San5\/P9js\/PFLykyfk\/\/rV4ubNeiHKxMQjZ\/uFs7+zHHtc\/vHTEZZUTLIbnyYnpc77bB1fje74S61ABWISkkW\/p6qw96\/tNbVa\/9H3C1omJp70\/d5vXrX8\/VdtALBjp3bhvPSjnywVvlSMmfSeLoxbLoxLOsA7IeuJlzl3WB2yGJ\/efx9am+wiL+9+4IbCFYsxkWm+JwF87\/H3274SLWhpmGTw9QgasCcasCcaWB955BHjzwtHf\/TeVzvkW\/5Mm5paeuVXs3\/3N87T\/yYvzL5611O\/Kz86AxdevTAkSQAgVdpvBIC2n\/\/tf5w7saduFwC89\/7cp5\/8q\/Mzf2y+uWk1jmU9k+A8d+ZLX4ye+T+QLAC6DiDpAJK09QFzOn7yS95P1N7uHf35I75e3\/3P127Y8uvg6a+dfPjFPU\/WbthSrPJ\/UEjQ75U9+DWQQNd1AEkCCSTJsqUm2ffvqP44APzm4msA8Iu3fbduvIUlFYIEefkNn\/q07PqLqN9v++zd9i+3W2pqrFtq5gHGZ4N\/fO\/C+GxwfDb4x\/eCn6i9HQBqN2zZ6dwRmHoD6uCNy2\/d85G7i34IHwgSnz\/ZPvu5qN8PNlvJbY3iza0barZuqPlErfnD93zk7h8OewLhcxfnJjG8mLyToN+L+l9ffPrflX889P7LpyL\/+z9pN3FH9cfn3lf\/9fc\/q67YzJ1egTDHk66q6mOH7F\/6sv2+r+jq3JVHvy27\/ty6aVOKTdRu2HLrxlue+P3xH3\/u+4Us6gcaczyp3\/4H\/eLFG3bdAwD2+\/ZK1dVX\/vmf0m7lno\/c7SzfxJ1e4TDHU8VjfeK1pCiVx5\/OcEM7nTu40ysc+bke8Z\/nXuJMr6Dk6unXwdObvlcHAG11X8xHeZjErGS+EVN8+DosDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDf4fbByyPd4nLFMAAAAASUVORK5CYII=","height":388,"width":141}}
%---
%[output:4a620c16]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAI0AAAGECAIAAACERDSOAAAAB3RJTUUH6gQaESsZvtAfkQAAG+ZJREFUeJztnX9sG9d9wL8UTyYlnWKlHmVTP+DUztqq4mZMLhslUYGqqAo1ae02m6OmqQVoXNJBrYA5NZxGwQCvqJU18Kyu6lwkHuvObZayWdbEWQa33sIiUVs5mtUaISEHSVSrpkxFdCwpOlpkRN7tj6\/1fD7+Fn9IX+X7gWFQFHn37n3u+9733r2nM4XDYWDWPGWrXQAmK9gTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDa57crvdjgS8Xm+qbyqK0tPT09PToyhKPiXw+\/2tra1utxt\/PHTokN\/vB4D+\/v7Ozs5QKJTxu1jUjB8WG9TvMf1esilDRkKhUGdnp6hScaQ5IYlXLpfL5XKFQqG9e\/fa7fahoSFZltN8U5bl48ePr2CXBpqbm0dGRvC12+1+4YUXvvCFL2TzRa\/X29fXt2\/fPpfLBQD9\/f2f\/\/zn3W53c3Nz9nsfGBhY8W+zwe\/3u1yuBx54AAuJZQYA\/DF70rV7eCL09\/fjDhwOh\/71qVOnRDzhO9\/73vfw7DacMvhbDM3+\/n79687OzuHhYTy7vV7v4OCgoihdXV0ijg8cOOBwOFpbWzHIBIqinDhxwul0dnV14TsDAwMjIyNCEu7I4XCkj3h9xCR+Rf9bfViIA8QPHDp0KFUhDx8+3NTUJArZ3t7u8\/mEJNGG6Qsp3hS79nq96TzZbLaWlpaxsbFQKDQxMQEAwWBQUZSJiYmGhoampibD56enp0dGRnbt2vXMM8\/o2wqHw9HQ0HD69GlFUYLBIABMTEzg65aWlptvvlkcw759+2RZ9ng87e3tABAIBLq7u71eb01NzVNPPaXf1+Tk5Pj4eFtbW9Kg7+\/vHxsb83q9Xq83GAxmExZer\/fkyZNDQ0Mej2d8fNzj8eh\/i81MS0uLz+cbGhoaHBwUZ1IgELjjjjtWUEg8L4eGhvSFdLvdx44d83g8IyMjdrv9wIED6C9DHtHR0TE3NzczM\/PWW28BwPj4+Pj4+PDwcEtLy6ZNmwwf3r59O\/6PXxHvo+9gMHj58mX0NDw8jJvq6OhIs3dZlmtraysqKux2O54i6UuLhEKhsbGxlpYWm81ms9n27Nnz0ksvGc70VPT19c3MzIyMjBjaJZ\/PFwgEsLROp9PpdJ44cQLLk2Uh9eGI0XP69OmGhgaHwyHqZ2ZmZnh4uKmpaevWrbIsd3d3j46Ojo6OQkZPtbW1APDcc8+NjY3dd999NTU1v\/3tb8fHx1GJgW3btqXazvbt28fHx0+ePDk3N\/fggw8Gg8Ff\/vKXYvupqKmpSf+BpMzMzMzNzSUtYRqw9gGgr68vMYGamJhAHzkVsra2tqamBk9xm8126tSpkZER3MvVq1eDwWAgEGhvb3c4HCdPngwGg++8804wGLTb7Ynxl8HT1q1bm5qaXn75ZU3TPvOZz9jt9ieffBIAWltbs60AAPH5l19+uamp6fbbb5+bm3v66afxxMlpO4aCDQ8PJ56\/+trJHkyLfD7fvn37AOA73\/mOvunetm2boij6RiIb0gRZZWWl3W5vaGjwer0+n8\/n8506daqxsTHV5zN4kmW5ra1tamrKZDLdcsstbW1tALCC+sVqPX\/+vN1ub2pqwr4tVcOdDaJZEB1Jf38\/9uT6bjUUCj3zzDOf+tSnMiaBbrcbv97V1eV0Ou12e0VFhfit6GIBANui7u7ujIWXZXn\/\/v3j4+OigxwYGMB2DAA6OjoCgYDP5xP5Gtb2+Pj45OSkSJQw\/qSUO1mmtbVVlmVs7rFlw8DM6bIJSzA6OtrR0SHLst1uh2TtZGtr67Fjx7q6uvCkTk97e7vH43G5XIODg7gLkZQPDAz09\/djMrJr165s8giXy\/XWW29hYoab0muw2Ww\/\/vGP9+7d63A4AGBoaAg3npHm5uYXXnhBfBEAGhoaHn\/8cVmWMW\/CNN3pdGIujf0iFsPpdIqrI1M4HM5mf8zqwuNGNGBPNGBPNMicR5AjFovh\/5IkSdI6OUBih4EO8AW+jkajABCPx\/H\/WCxmNpsBwGQyAYCmaaqqSpKkaZrZbJYkyWw2a5omFFJxubaKKEJBvMCqN5lM4kVZWVksFltcXKyqqiovL8datlgsot7Fm7iFpaUlwzZjsVgkEonFYuFwuKKioqysDL+IRs1mM5peUy5Ll5cnOsB6RxkYEKgBX5hMpvLycrixvvAdAAgGg42NjVardcWFiUQigUBgy5YtkiQldalpmqZpsByUAICxiCINCovtsjCbTuoAdM2R0KCq6uLiYmVlZUVFhclkkiQJ61o4kJZJv7s8KwV3YbFYZFlOtSlxLJAiLpeWlgxBiRYBoKysLFFhPi4zfy2VA300YGcgOgZJkjAacOhF7yAWi+UZCiVDVG6az+DdGZvNZrVaE12iSIxLjEh8re8vs3QpTU1NxWKxzZs346YjkQgkc2CIg\/LycuwG9A7S7EaQfyisKfB40wQlpI1LTdPwnaWlJUVRqqqqysrKsLHV5zuSJEnPP\/\/8K6+8sn\/\/frvdLuJAdMUiIBYXFy9fvkwiDn59dvgHv\/qXDZ7btm9wvPPI811\/ev9tH8ptdL+w5BOX+GJpaUmqq6t77bXXTCbTrbfems3+1j7n5n\/XvPvDZXcoP\/zJA3OHFzbfV7+6nrIBY8ZqtaYKg7Jrw+ZEHOgR11KJaAAaaKUsTLFZr+NGmSWl0bwGWR1PedZRAaOfSkOyXuOpRJQsKEvticr5mxMlOCiOJxpc8zQ1NbW65SgKJjCtdhEKRVl9ff1ql4HJDOF2D6\/Vk6LBtaHulX19DbIOe3UAUONxk4ZXu+kglNSQKaiB9FUcV+NaRkuFoGRBuTqein14cTWuqUXdQ6lZBU8laG1K6ak0jSfVdi89cTWurqth2GVPgUBgdctRWOJq3KRqkCnlI8Q6jad43LSOJAF6qqurW+1irIQ0Y6BxNbMnWvc11mM8acvt3jpidTzlfy6nzbK0eDwOqprx+in\/VK1kQbke4wkgrsZBWzdjsADr+PqpZJ74+mnlrNvxCLrXT7FlFhcX8cfZubn4xrgWu9Y9KYoSCoXEdMS1P\/8wKZTiSa\/kypUrAKBfy40mrFarZJbi8bgWv\/5+LBbDT87NzYk3Z2dnLRaL1WqtqKhYCysy0rNGC5cYJTihGsGqlySppqYGp+7qo0SuluPRuLrsyWq14vJ6MTEbR4GvXLkyPz+\/YcMGvTyxKZwIvHaCTwKA1b2lq1ein+OOiPnVNpsNf8S6w\/m94m8jGVDjqho3zrTEiBF\/bQD3Yrfb9SdEJBJRFCUWixnkiZn0hnOiZJQunsRE+KtXr8Z0XC9KCiU5o4Gqqmo883Wufu2CYV8GeaLl1H\/RarVeuXLFbDZHIpFiyyuKJ31YYJTolbzzzjuSJN10003YN0A+SlKgxlVVzWvychp5YuVTLBabn5+H5QVF1zvIIuQshbkgT6VEnLCiuwYAWZZjsRj2GUVCVTUUVVhSpRs2mw27PTz2xJxF3+2tLGfJ4Qv6QmSjJFWBFhYWci1lrqiqqsYzjxsVBLEESv+mIWeJRCKoLZU8fZecfC9J302Mkvn5+VAoFI\/HxYplDPD0SlYLVVXVVb3QNeQsglQ5SzQaxeqtqqpKmrNIR48enZqaUlV1dnYWt2JItwDAarViY7Vly5b0a+dKScoxUA00jCf8YS2RqttbWFiIRqOyLOMKw8ScRXI6nUePHrXZbPg7VJIYJRjCopNc48TjqpppvDy2ZlagYrN000031dTUCIWGnEVyOp27d++enp5ubGxc3eLmRPoq1sXTyjeyuhi6EpLrnzKhqaqqlmR+RMnuPxGet5wKDUCNa6oYOFoXrIKnorc2OB6Rxf3cglCaxnMdxhNc65\/W1YSj9ekJxyNWuxSFhPA6tTTXT9favZV9fU2yXuOpdONGpYFqXi7+wFtSVFXNJt9Lv5E1RRms9n3CgmOuMGuYRGgAGkgV5uLtaz1fPxX9LNY0AE2Lq7DWRvfyYH32T1oJ12qUpvFcn54yL82lxnr0tCxJ01Led6eVlIPwRHSeZdLq1pb\/vGfpy1M81mM8YSRB5qZvLd\/XMEDVU7oq1q7FVLHLUMrGswxWaT1hcQ8S87111PJRjaf0aHA9MS+2rHV7X6MEB7bu0oi1ug4gJ8QsNpzCNjs7q79+UhYWZmdngfKiGqDoCa3gqqaKigpN08REtmsz3yTp2h8MW44p\/RxH\/ExZWdns7Kx+fs8ah4AnMaUQw0U8sSAej\/\/hD3+orKysrKyUZRnn4eK8Nn0SIVfLjY2NOEcOZzSGw+GFhQVJki5fvjw3Nyfm1K3icoyMXPcUCAQaGhpWsSiIYZmNPlYAwGq12mw2rNkPfvCDOAU6sgxGzOzs3LWhCA1AA2VBuXjxIm4E5x9u2rRJTHYXU1PFjGLQzYZcO+ZW+e8SZGMFUi\/owPVP+EV8REU4HI5Govp4CofD0Wi0qqpKP39UbEG\/WcOkYoM5dKw3Z1gXVFRK6klUxMLCwquvvgoAkiRZLBaLxVJVVaWfipvmFE6vdtOmTRs3brw2IKEBAGzcuHHz5s1Y7\/pp+GJ+\/PU53AmTihPN4ePbotEoPifGYrHgZ4qdxBZxPaEhDdOffR\/4wAei0Sge29LSEh4t1ri0\/NQs\/TmbU8BZrVYhCX8Uyw71m8Jp3CK5gOVp+8JcTPccJdw1fkw8Q8xsNquqKh4vntR9oSjYWaC3AjeuqBXnaapY0Tdcb7\/9diwWi0aj+KA7XB6SU8ABpLwBlTRicNcYLvh0OX24AEBlZWVNTU2qXad3Xyhzua1\/MvyYmIbBjcs3M7ZgkBBwFovFbDZbrdZ4PJ4YcBht+oBLiqYuD8WmHo4w7B3LX1VVhbGi3zUAKIoiSVIsFrNarYuLixlbS8OytTRRK76SvqvLwZOqqq+\/\/jo+Lg0PQL8WSqRhqVrqggScYcFX8qZGW35gjWbcSKomVJKkNOECy2vrEveeNC2Uki1bE+XHNU+XL1\/GZ6BFo1GLxaIoSvoFS1LSmXtJDwmfuYcfMJvNeACippIeYZ4Bd72UNx65oXvXn7CSJEUii+LOBiw\/AitVfp9x7\/h5\/cL6xORC\/2HD8j3D++Xl5VgbaAtbVwCwWCxlZWUXL17U1+QNUTs6Onr06FFYvsKPJaxTS9UrxGIxcZYpiiJOEJPJhGMB2GJkH3DZk7SpWVhYWHY2fy3b0zQACCvh+fn5qqqqm2++OcvTYgV7TzxvYstPDdRXiKgNfHqdfrUTLC8ySxq1kkj27rrrri1btuzYsWPLli133nlnW1tbxuQYzw7xJqZw0WgUy4SdDT4jrOApUNLeBatgd8sXm7c5QnMz8Yj6J+W2j97yUSwM\/vWcpGdrnojzD+tEkiRFUeLxuMhHDGctdhz681XfdYkDDAQCb7755oULF86ePXvt+blTU1PPP\/88AIyOjo6OjuJHGxoabrvtNvz\/Yx\/7WGKlgC4hNgScPtqS9kO51lH63kU0Nak2q+8hEstv7OGyLpLIF\/BvTICu\/UisEFEAfdagP3Xm5+enp6enpqZGRkYCgcCZM2dg+e5gkuccY4+VqE1E286dOz\/5yU\/m1IYkrab02vArSRfWZ1xSn0159Ml0YpGSjhilOYqMJ0rSMmC4XLhw4dy5c9PT0+fOnYNlMU6n0+l01tfX44MJMz+Pempq6tKlS2grVbS1tub2oMbEA8ZOFZ8BCwD42OB8aiFXkjYAsVgMHyZuaLtWXKRAIBAIBDBizpw5g9OHhBi0gmIM5Pzc8FTRlqs2faOB43LYoMduvLaVZbm8vLxk67dFEEciERwYRFX4WGK8FMGx+SxLhc0X6kkUo4+Y9OT7fPfstenbsfSNRqrmBVJcJOZDlqUyXEUlLVV1dbUkScIHRgx+rK6urr6+vq6uLnsxBvL1ZCCNtp07d9bU1OzYsePOO++EhLwjDaIvyV9bqixRLybL2BWlevPNN8+ePXvu3LnLly+fPXsWf5u\/GAMF9mRAr+255567dOkSvp9n35aoDXTXwnptacTos\/OcGlWMGH1KhqCVQokxYAqHw6FQaO\/evdh07tu3z+VyAYDb7R4cHASAoaGh9vb2\/PeE4bV79+78+zY9Bm0oJhqNqqpqsVgikYgY31px\/p1UjCFiRkdHU6UAK0bvxRQOh\/v7+7dv3+5yufx+v8vleuyxx2praw8ePHj06FGfz3fixImhoaHEP9WTPwVJSQwRo7\/AxEFITEmSRltSsF7Si0E3+R5\/JvRebmj3FEXp6+vr7u6emJgYHh4eGhpaXFzs7e09ePBgc3NzsYuVf0qS9OpSXCcZ\/mgQhld1dfX09DSkEIPxUTIxqVAU5YZ2eXJyUlEUh8MxMTFht9tlWcZ7rzMzMyXwhBXR29uLPxq0Pfvss\/i+ISVJP5yqv9q32WwYfOfPnweAF198MbHzdzqdvb29BW\/B8mRycvK6p1AodPDgwYMHD+Id0lUnlTYxxIVk00imasqQNShGD3qRvF5ve3u73+\/HDklICgaD2FBUV1fX1taubB8FTEaEtqmpKUyo6urqkkab0GYYKIPliPn2t7+NW0Dlu3fvLlSb5vV6+\/r6QJeO5Up\/f\/\/JkydhucaEF1M4HPb7\/U888cTAwIBIFsSv88kjCrKRLEnVtxU1VzYQCoWwLwcAw0m\/MvReJEVRDh8+PDo6KhoNNNnZ2dne3i7LstvtXln9joyM4I0rh8OhKMrk5GTxOrmkjWSJe36fz7ewsFBbW1tRUSHLss\/ny6cJMXiRZFk+fvx44udcLtfKIldP6ZMRZLVyM7vdjn9BGgAmJiby8WTwsj7X1aw\/iutpBclIKBTq7Ox0OBwOh8PtduObbrcb3\/F6vQUvpKIoPT09uP3+\/n5DMXp6evQXXukJBoPiBve2bdsKWMgiemptbVUUZXFx0efzybK8devWbL41ODi4Z88en8\/n8XiOHTvm9Xr9fv+pU6e8Xu\/Q0NCJEyeyr7Us8Xg8drvd5\/N5vd6xsTE8ObAYIyMjACCykvQ4HI7q6uqZmRlxGVrAQhbxpk5zc\/MKkpGBgQF8sXXr1qamJih+PiK6YZvN1tLSAgChUGhsbKyjo0OW5ba2ttOnT2fT09hstt7e3q6uLgAYGhoq7GVocW++5ZOMlH5wJBQKvfHGG\/fffz\/c2FBj653Nedbe3u7z+YpRtjWaR5R+cERRlAMHDvT29pYsKc2JtejJ7\/f39vYePXpUVFlBBkfSEAqFenp69u\/fL9o3DFx8jdFc8J3mxJrzhBfhx48fF5G0snwke0Kh0IEDB\/SnBXZUExMTiqIMDw93dHQUdo8roLj3c3MFb6zo8yscHMFxQsxHCt4uiSE1BIfmxD26Xbt2idRmFcnBExZ9z549IjVIfIfJiNfrfeSRR8QJJ+7Npk8pc4snv9\/\/0EMPHTlyBPeBV4Vr4XSjhdvtxtuwANDX19fW1pbxRM+tf2pubr733nsPHz6sKIrf73\/jjTf27du38vK+X8FrLI\/H4\/F4xI\/pybl\/wrbu61\/\/+rPPPtvd3V2QKS7vQ7C5A4Ase9yc8z2bzfbwww9\/85vfBIA1ew907YOjLU1NTVmmryvJy\/HOW3d396pfVdDF4\/EoiqIoCjZ9GSHw91jWH36\/\/2c\/+9mRI0cA4KGHHmptbc3Y9K2569x1TygU+sY3vnHvvfc2Nzfr87L032JPpebAgQN2u13kePgi47XN2hqPYFLB8UQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kSDbD394r\/N9qpK\/b\/vHykvaskYPaZwOJzrd75\/pPxHT0r\/9avIli1aMcrEJCLl+oVzvys7\/oT0w59GWVIpya1\/mp42ue6z9Hw1tuMv1CIViElKDu2eosDev7I2NKpDx94rapmYRDK3e795pezvvmoBgB071YsXTD\/40VLxS8UYyezp4mTZxUmTBvB20HzyJc4dVocc+qf33oPONqvIy\/se2FC8YjEGss33TADffeK9rq\/EiloaJhU8HkED9kQD9kQD86OPPqr\/efHYD979ao9064fUmZmll381\/7d\/bT\/zb9Li\/Ct3Pf27qmNzcPGViyMmEwCYaqw3AUDXz\/\/mP86f3NO0CwDefW\/h00\/95YW5P7bf0rYax7KeSXKdO\/elL8bO\/h+YygA0DcCkAZhMWx8wpuOnvuT5ROPtnvGfP+od8N7\/XOPG+l8Hznzt1MMv7HmqcWN9qcr\/fiFJu1f54NfABJqmAZhMYAKTqay+IdX376j7OAD85tKrAPCLt7wf3nQrSyoGSfLyDZ\/6tOT485jPZ\/ns3dYvd5c1NJjrG8IAk\/OBP757cXI+MDkf+OO7gU803g4AjRvrd9p3+GdehyZ4\/cqb93zk7pIfwvuC5NdPls9+LubzgcVSflureHPrxoatGxs+0Wj88D0fufv7o25\/6PylhWkML6bgJGn3Yr7XIj\/9d\/kfDr330uno\/\/5Pxk3cUffxhfeUf\/39T+qqt3CjVySM8aQpivL4IeuXvmy97yuasnD1sW9Jjj8zb96cZhONG+s\/vOnWJ39\/4oef++diFvV9jTGelG\/9vXbp0oZd9wCA9b69prq6q\/\/0jxm3cs9H7rZXbeZGr3gY46n68UHx2iTLNSd+muWGdtp3cKNXPAozHvGf51\/kTK+o5Ovp14Ezm7\/bBABdTV8sRHmY5KxkvhFTengclgbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQb\/Dwl\/q+HvbXtmAAAAAElFTkSuQmCC","height":388,"width":141}}
%---
%[output:76b94fa5]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAI0AAAGECAIAAACERDSOAAAAB3RJTUUH6gQaESsZvtAfkQAAG+ZJREFUeJztnX9sG9d9wL8UTyYlnWKlHmVTP+DUztqq4mZMLhslUYGqqAo1ae02m6OmqQVoXNJBrYA5NZxGwQCvqJU18Kyu6lwkHuvObZayWdbEWQa33sIiUVs5mtUaISEHSVSrpkxFdCwpOlpkRN7tj6\/1fD7+Fn9IX+X7gWFQFHn37n3u+9733r2nM4XDYWDWPGWrXQAmK9gTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDa57crvdjgS8Xm+qbyqK0tPT09PToyhKPiXw+\/2tra1utxt\/PHTokN\/vB4D+\/v7Ozs5QKJTxu1jUjB8WG9TvMf1esilDRkKhUGdnp6hScaQ5IYlXLpfL5XKFQqG9e\/fa7fahoSFZltN8U5bl48ePr2CXBpqbm0dGRvC12+1+4YUXvvCFL2TzRa\/X29fXt2\/fPpfLBQD9\/f2f\/\/zn3W53c3Nz9nsfGBhY8W+zwe\/3u1yuBx54AAuJZQYA\/DF70rV7eCL09\/fjDhwOh\/71qVOnRDzhO9\/73vfw7DacMvhbDM3+\/n79687OzuHhYTy7vV7v4OCgoihdXV0ijg8cOOBwOFpbWzHIBIqinDhxwul0dnV14TsDAwMjIyNCEu7I4XCkj3h9xCR+Rf9bfViIA8QPHDp0KFUhDx8+3NTUJArZ3t7u8\/mEJNGG6Qsp3hS79nq96TzZbLaWlpaxsbFQKDQxMQEAwWBQUZSJiYmGhoampibD56enp0dGRnbt2vXMM8\/o2wqHw9HQ0HD69GlFUYLBIABMTEzg65aWlptvvlkcw759+2RZ9ng87e3tABAIBLq7u71eb01NzVNPPaXf1+Tk5Pj4eFtbW9Kg7+\/vHxsb83q9Xq83GAxmExZer\/fkyZNDQ0Mej2d8fNzj8eh\/i81MS0uLz+cbGhoaHBwUZ1IgELjjjjtWUEg8L4eGhvSFdLvdx44d83g8IyMjdrv9wIED6C9DHtHR0TE3NzczM\/PWW28BwPj4+Pj4+PDwcEtLy6ZNmwwf3r59O\/6PXxHvo+9gMHj58mX0NDw8jJvq6OhIs3dZlmtraysqKux2O54i6UuLhEKhsbGxlpYWm81ms9n27Nnz0ksvGc70VPT19c3MzIyMjBjaJZ\/PFwgEsLROp9PpdJ44cQLLk2Uh9eGI0XP69OmGhgaHwyHqZ2ZmZnh4uKmpaevWrbIsd3d3j46Ojo6OQkZPtbW1APDcc8+NjY3dd999NTU1v\/3tb8fHx1GJgW3btqXazvbt28fHx0+ePDk3N\/fggw8Gg8Ff\/vKXYvupqKmpSf+BpMzMzMzNzSUtYRqw9gGgr68vMYGamJhAHzkVsra2tqamBk9xm8126tSpkZER3MvVq1eDwWAgEGhvb3c4HCdPngwGg++8804wGLTb7Ynxl8HT1q1bm5qaXn75ZU3TPvOZz9jt9ieffBIAWltbs60AAPH5l19+uamp6fbbb5+bm3v66afxxMlpO4aCDQ8PJ56\/+trJHkyLfD7fvn37AOA73\/mOvunetm2boij6RiIb0gRZZWWl3W5vaGjwer0+n8\/n8506daqxsTHV5zN4kmW5ra1tamrKZDLdcsstbW1tALCC+sVqPX\/+vN1ub2pqwr4tVcOdDaJZEB1Jf38\/9uT6bjUUCj3zzDOf+tSnMiaBbrcbv97V1eV0Ou12e0VFhfit6GIBANui7u7ujIWXZXn\/\/v3j4+OigxwYGMB2DAA6OjoCgYDP5xP5Gtb2+Pj45OSkSJQw\/qSUO1mmtbVVlmVs7rFlw8DM6bIJSzA6OtrR0SHLst1uh2TtZGtr67Fjx7q6uvCkTk97e7vH43G5XIODg7gLkZQPDAz09\/djMrJr165s8giXy\/XWW29hYoab0muw2Ww\/\/vGP9+7d63A4AGBoaAg3npHm5uYXXnhBfBEAGhoaHn\/8cVmWMW\/CNN3pdGIujf0iFsPpdIqrI1M4HM5mf8zqwuNGNGBPNGBPNMicR5AjFovh\/5IkSdI6OUBih4EO8AW+jkajABCPx\/H\/WCxmNpsBwGQyAYCmaaqqSpKkaZrZbJYkyWw2a5omFFJxubaKKEJBvMCqN5lM4kVZWVksFltcXKyqqiovL8datlgsot7Fm7iFpaUlwzZjsVgkEonFYuFwuKKioqysDL+IRs1mM5peUy5Ll5cnOsB6RxkYEKgBX5hMpvLycrixvvAdAAgGg42NjVardcWFiUQigUBgy5YtkiQldalpmqZpsByUAICxiCINCovtsjCbTuoAdM2R0KCq6uLiYmVlZUVFhclkkiQJ61o4kJZJv7s8KwV3YbFYZFlOtSlxLJAiLpeWlgxBiRYBoKysLFFhPi4zfy2VA300YGcgOgZJkjAacOhF7yAWi+UZCiVDVG6az+DdGZvNZrVaE12iSIxLjEh8re8vs3QpTU1NxWKxzZs346YjkQgkc2CIg\/LycuwG9A7S7EaQfyisKfB40wQlpI1LTdPwnaWlJUVRqqqqysrKsLHV5zuSJEnPP\/\/8K6+8sn\/\/frvdLuJAdMUiIBYXFy9fvkwiDn59dvgHv\/qXDZ7btm9wvPPI811\/ev9tH8ptdL+w5BOX+GJpaUmqq6t77bXXTCbTrbfems3+1j7n5n\/XvPvDZXcoP\/zJA3OHFzbfV7+6nrIBY8ZqtaYKg7Jrw+ZEHOgR11KJaAAaaKUsTLFZr+NGmSWl0bwGWR1PedZRAaOfSkOyXuOpRJQsKEvticr5mxMlOCiOJxpc8zQ1NbW65SgKJjCtdhEKRVl9ff1ql4HJDOF2D6\/Vk6LBtaHulX19DbIOe3UAUONxk4ZXu+kglNSQKaiB9FUcV+NaRkuFoGRBuTqein14cTWuqUXdQ6lZBU8laG1K6ak0jSfVdi89cTWurqth2GVPgUBgdctRWOJq3KRqkCnlI8Q6jad43LSOJAF6qqurW+1irIQ0Y6BxNbMnWvc11mM8acvt3jpidTzlfy6nzbK0eDwOqprx+in\/VK1kQbke4wkgrsZBWzdjsADr+PqpZJ74+mnlrNvxCLrXT7FlFhcX8cfZubn4xrgWu9Y9KYoSCoXEdMS1P\/8wKZTiSa\/kypUrAKBfy40mrFarZJbi8bgWv\/5+LBbDT87NzYk3Z2dnLRaL1WqtqKhYCysy0rNGC5cYJTihGsGqlySppqYGp+7qo0SuluPRuLrsyWq14vJ6MTEbR4GvXLkyPz+\/YcMGvTyxKZwIvHaCTwKA1b2lq1ein+OOiPnVNpsNf8S6w\/m94m8jGVDjqho3zrTEiBF\/bQD3Yrfb9SdEJBJRFCUWixnkiZn0hnOiZJQunsRE+KtXr8Z0XC9KCiU5o4Gqqmo883Wufu2CYV8GeaLl1H\/RarVeuXLFbDZHIpFiyyuKJ31YYJTolbzzzjuSJN10003YN0A+SlKgxlVVzWvychp5YuVTLBabn5+H5QVF1zvIIuQshbkgT6VEnLCiuwYAWZZjsRj2GUVCVTUUVVhSpRs2mw27PTz2xJxF3+2tLGfJ4Qv6QmSjJFWBFhYWci1lrqiqqsYzjxsVBLEESv+mIWeJRCKoLZU8fZecfC9J302Mkvn5+VAoFI\/HxYplDPD0SlYLVVXVVb3QNeQsglQ5SzQaxeqtqqpKmrNIR48enZqaUlV1dnYWt2JItwDAarViY7Vly5b0a+dKScoxUA00jCf8YS2RqttbWFiIRqOyLOMKw8ScRXI6nUePHrXZbPg7VJIYJRjCopNc48TjqpppvDy2ZlagYrN000031dTUCIWGnEVyOp27d++enp5ubGxc3eLmRPoq1sXTyjeyuhi6EpLrnzKhqaqqlmR+RMnuPxGet5wKDUCNa6oYOFoXrIKnorc2OB6Rxf3cglCaxnMdxhNc65\/W1YSj9ekJxyNWuxSFhPA6tTTXT9favZV9fU2yXuOpdONGpYFqXi7+wFtSVFXNJt9Lv5E1RRms9n3CgmOuMGuYRGgAGkgV5uLtaz1fPxX9LNY0AE2Lq7DWRvfyYH32T1oJ12qUpvFcn54yL82lxnr0tCxJ01Led6eVlIPwRHSeZdLq1pb\/vGfpy1M81mM8YSRB5qZvLd\/XMEDVU7oq1q7FVLHLUMrGswxWaT1hcQ8S87111PJRjaf0aHA9MS+2rHV7X6MEB7bu0oi1ug4gJ8QsNpzCNjs7q79+UhYWZmdngfKiGqDoCa3gqqaKigpN08REtmsz3yTp2h8MW44p\/RxH\/ExZWdns7Kx+fs8ah4AnMaUQw0U8sSAej\/\/hD3+orKysrKyUZRnn4eK8Nn0SIVfLjY2NOEcOZzSGw+GFhQVJki5fvjw3Nyfm1K3icoyMXPcUCAQaGhpWsSiIYZmNPlYAwGq12mw2rNkPfvCDOAU6sgxGzOzs3LWhCA1AA2VBuXjxIm4E5x9u2rRJTHYXU1PFjGLQzYZcO+ZW+e8SZGMFUi\/owPVP+EV8REU4HI5Govp4CofD0Wi0qqpKP39UbEG\/WcOkYoM5dKw3Z1gXVFRK6klUxMLCwquvvgoAkiRZLBaLxVJVVaWfipvmFE6vdtOmTRs3brw2IKEBAGzcuHHz5s1Y7\/pp+GJ+\/PU53AmTihPN4ePbotEoPifGYrHgZ4qdxBZxPaEhDdOffR\/4wAei0Sge29LSEh4t1ri0\/NQs\/TmbU8BZrVYhCX8Uyw71m8Jp3CK5gOVp+8JcTPccJdw1fkw8Q8xsNquqKh4vntR9oSjYWaC3AjeuqBXnaapY0Tdcb7\/9diwWi0aj+KA7XB6SU8ABpLwBlTRicNcYLvh0OX24AEBlZWVNTU2qXad3Xyhzua1\/MvyYmIbBjcs3M7ZgkBBwFovFbDZbrdZ4PJ4YcBht+oBLiqYuD8WmHo4w7B3LX1VVhbGi3zUAKIoiSVIsFrNarYuLixlbS8OytTRRK76SvqvLwZOqqq+\/\/jo+Lg0PQL8WSqRhqVrqggScYcFX8qZGW35gjWbcSKomVJKkNOECy2vrEveeNC2Uki1bE+XHNU+XL1\/GZ6BFo1GLxaIoSvoFS1LSmXtJDwmfuYcfMJvNeACippIeYZ4Bd72UNx65oXvXn7CSJEUii+LOBiw\/AitVfp9x7\/h5\/cL6xORC\/2HD8j3D++Xl5VgbaAtbVwCwWCxlZWUXL17U1+QNUTs6Onr06FFYvsKPJaxTS9UrxGIxcZYpiiJOEJPJhGMB2GJkH3DZk7SpWVhYWHY2fy3b0zQACCvh+fn5qqqqm2++OcvTYgV7TzxvYstPDdRXiKgNfHqdfrUTLC8ySxq1kkj27rrrri1btuzYsWPLli133nlnW1tbxuQYzw7xJqZw0WgUy4SdDT4jrOApUNLeBatgd8sXm7c5QnMz8Yj6J+W2j97yUSwM\/vWcpGdrnojzD+tEkiRFUeLxuMhHDGctdhz681XfdYkDDAQCb7755oULF86ePXvt+blTU1PPP\/88AIyOjo6OjuJHGxoabrvtNvz\/Yx\/7WGKlgC4hNgScPtqS9kO51lH63kU0Nak2q+8hEstv7OGyLpLIF\/BvTICu\/UisEFEAfdagP3Xm5+enp6enpqZGRkYCgcCZM2dg+e5gkuccY4+VqE1E286dOz\/5yU\/m1IYkrab02vArSRfWZ1xSn0159Ml0YpGSjhilOYqMJ0rSMmC4XLhw4dy5c9PT0+fOnYNlMU6n0+l01tfX44MJMz+Pempq6tKlS2grVbS1tub2oMbEA8ZOFZ8BCwD42OB8aiFXkjYAsVgMHyZuaLtWXKRAIBAIBDBizpw5g9OHhBi0gmIM5Pzc8FTRlqs2faOB43LYoMduvLaVZbm8vLxk67dFEEciERwYRFX4WGK8FMGx+SxLhc0X6kkUo4+Y9OT7fPfstenbsfSNRqrmBVJcJOZDlqUyXEUlLVV1dbUkScIHRgx+rK6urr6+vq6uLnsxBvL1ZCCNtp07d9bU1OzYsePOO++EhLwjDaIvyV9bqixRLybL2BWlevPNN8+ePXvu3LnLly+fPXsWf5u\/GAMF9mRAr+255567dOkSvp9n35aoDXTXwnptacTos\/OcGlWMGH1KhqCVQokxYAqHw6FQaO\/evdh07tu3z+VyAYDb7R4cHASAoaGh9vb2\/PeE4bV79+78+zY9Bm0oJhqNqqpqsVgikYgY31px\/p1UjCFiRkdHU6UAK0bvxRQOh\/v7+7dv3+5yufx+v8vleuyxx2praw8ePHj06FGfz3fixImhoaHEP9WTPwVJSQwRo7\/AxEFITEmSRltSsF7Si0E3+R5\/JvRebmj3FEXp6+vr7u6emJgYHh4eGhpaXFzs7e09ePBgc3NzsYuVf0qS9OpSXCcZ\/mgQhld1dfX09DSkEIPxUTIxqVAU5YZ2eXJyUlEUh8MxMTFht9tlWcZ7rzMzMyXwhBXR29uLPxq0Pfvss\/i+ISVJP5yqv9q32WwYfOfPnweAF198MbHzdzqdvb29BW\/B8mRycvK6p1AodPDgwYMHD+Id0lUnlTYxxIVk00imasqQNShGD3qRvF5ve3u73+\/HDklICgaD2FBUV1fX1taubB8FTEaEtqmpKUyo6urqkkab0GYYKIPliPn2t7+NW0Dlu3fvLlSb5vV6+\/r6QJeO5Up\/f\/\/JkydhucaEF1M4HPb7\/U888cTAwIBIFsSv88kjCrKRLEnVtxU1VzYQCoWwLwcAw0m\/MvReJEVRDh8+PDo6KhoNNNnZ2dne3i7LstvtXln9joyM4I0rh8OhKMrk5GTxOrmkjWSJe36fz7ewsFBbW1tRUSHLss\/ny6cJMXiRZFk+fvx44udcLtfKIldP6ZMRZLVyM7vdjn9BGgAmJiby8WTwsj7X1aw\/iutpBclIKBTq7Ox0OBwOh8PtduObbrcb3\/F6vQUvpKIoPT09uP3+\/n5DMXp6evQXXukJBoPiBve2bdsKWMgiemptbVUUZXFx0efzybK8devWbL41ODi4Z88en8\/n8XiOHTvm9Xr9fv+pU6e8Xu\/Q0NCJEyeyr7Us8Xg8drvd5\/N5vd6xsTE8ObAYIyMjACCykvQ4HI7q6uqZmRlxGVrAQhbxpk5zc\/MKkpGBgQF8sXXr1qamJih+PiK6YZvN1tLSAgChUGhsbKyjo0OW5ba2ttOnT2fT09hstt7e3q6uLgAYGhoq7GVocW++5ZOMlH5wJBQKvfHGG\/fffz\/c2FBj653Nedbe3u7z+YpRtjWaR5R+cERRlAMHDvT29pYsKc2JtejJ7\/f39vYePXpUVFlBBkfSEAqFenp69u\/fL9o3DFx8jdFc8J3mxJrzhBfhx48fF5G0snwke0Kh0IEDB\/SnBXZUExMTiqIMDw93dHQUdo8roLj3c3MFb6zo8yscHMFxQsxHCt4uiSE1BIfmxD26Xbt2idRmFcnBExZ9z549IjVIfIfJiNfrfeSRR8QJJ+7Npk8pc4snv9\/\/0EMPHTlyBPeBV4Vr4XSjhdvtxtuwANDX19fW1pbxRM+tf2pubr733nsPHz6sKIrf73\/jjTf27du38vK+X8FrLI\/H4\/F4xI\/pybl\/wrbu61\/\/+rPPPtvd3V2QKS7vQ7C5A4Ase9yc8z2bzfbwww9\/85vfBIA1ew907YOjLU1NTVmmryvJy\/HOW3d396pfVdDF4\/EoiqIoCjZ9GSHw91jWH36\/\/2c\/+9mRI0cA4KGHHmptbc3Y9K2569x1TygU+sY3vnHvvfc2Nzfr87L032JPpebAgQN2u13kePgi47XN2hqPYFLB8UQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kSDbD394r\/N9qpK\/b\/vHykvaskYPaZwOJzrd75\/pPxHT0r\/9avIli1aMcrEJCLl+oVzvys7\/oT0w59GWVIpya1\/mp42ue6z9Hw1tuMv1CIViElKDu2eosDev7I2NKpDx94rapmYRDK3e795pezvvmoBgB071YsXTD\/40VLxS8UYyezp4mTZxUmTBvB20HzyJc4dVocc+qf33oPONqvIy\/se2FC8YjEGss33TADffeK9rq\/EiloaJhU8HkED9kQD9kQD86OPPqr\/efHYD979ao9064fUmZmll381\/7d\/bT\/zb9Li\/Ct3Pf27qmNzcPGViyMmEwCYaqw3AUDXz\/\/mP86f3NO0CwDefW\/h00\/95YW5P7bf0rYax7KeSXKdO\/elL8bO\/h+YygA0DcCkAZhMWx8wpuOnvuT5ROPtnvGfP+od8N7\/XOPG+l8Hznzt1MMv7HmqcWN9qcr\/fiFJu1f54NfABJqmAZhMYAKTqay+IdX376j7OAD85tKrAPCLt7wf3nQrSyoGSfLyDZ\/6tOT485jPZ\/ns3dYvd5c1NJjrG8IAk\/OBP757cXI+MDkf+OO7gU803g4AjRvrd9p3+GdehyZ4\/cqb93zk7pIfwvuC5NdPls9+LubzgcVSflureHPrxoatGxs+0Wj88D0fufv7o25\/6PylhWkML6bgJGn3Yr7XIj\/9d\/kfDr330uno\/\/5Pxk3cUffxhfeUf\/39T+qqt3CjVySM8aQpivL4IeuXvmy97yuasnD1sW9Jjj8zb96cZhONG+s\/vOnWJ39\/4oef++diFvV9jTGelG\/9vXbp0oZd9wCA9b69prq6q\/\/0jxm3cs9H7rZXbeZGr3gY46n68UHx2iTLNSd+muWGdtp3cKNXPAozHvGf51\/kTK+o5Ovp14Ezm7\/bBABdTV8sRHmY5KxkvhFTengclgbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQbsiQb\/Dwl\/q+HvbXtmAAAAAElFTkSuQmCC","height":388,"width":141}}
%---
%[output:48bd1345]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAI0AAAGECAIAAACERDSOAAAAB3RJTUUH6gQaESsZvtAfkQAAG\/9JREFUeJztnX9sG9d9wL8UTyYlnWIlHmVTluDUDtKq4mZMLhslUYGqmAo1ae02m6OmqQVoXNJBrYA5NZxUwQCvqJU1cK2uDFwkHuvOXZqyWVbHWQa33sIiUVs5mtUaISEHSVSrpkxFdCwpOllSRN7tj6\/1fD7+Fn+IX+b7gWGQFO\/u3fvc973v3b3HM83PzwNT9JStdQGYtGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNGBPNLjuyePxOGLw+XyJllQUpbu7u7u7W1GUbEoQCARaWlo8Hg++PXjwYCAQAIC+vr6Ojo5wOJxyWSxqyi+LFeq3mHwr6ZQhJeFwuKOjQ1Sp2NOMkMQrl8vlcrnC4fCePXvsdrvb7ZZlOcmSsiwfO3ZsFZs00NTUNDQ0hK89Hs9LL730xS9+MZ0FfT5fb2\/v3r17XS4XAPT19X3hC1\/weDxNTU3pb72\/v3\/Vf02HQCDgcrkeeughLCSWGQDwbfoka\/fwQOjr68MNOBwO\/etTp06JeMJPfvCDH+DRbThk8K8Ymn19ffrXHR0dg4ODeHT7fL6BgQFFUTo7O0Uc79+\/3+FwtLS0YJAJFEU5fvy40+ns7OzET\/r7+4eGhoQk3JDD4Uge8fqIiV1E\/1d9WIgdxC8cPHgwUSEPHTrU2NgoCtnW1ub3+4Uk0YbpCyk+FJv2+XzJPNlstubm5pGRkXA4PDY2BgChUEhRlLGxsfr6+sbGRsP3Jycnh4aGdu7c+fzzz+vbCofDUV9ff\/r0aUVRQqEQAIyNjeHr5ubmm2++WezD3r17ZVn2er1tbW0AEAwGu7q6fD5fTU3Ns88+q9\/W+Pj46Ohoa2tr3KDv6+sbGRnx+Xw+ny8UCqUTFj6f7+TJk2632+v1jo6Oer1e\/V+xmWlubvb7\/W63e2BgQBxJwWDwrrvuWkUh8bh0u936Qno8nqNHj3q93qGhIbvdvn\/\/fvSXIo9ob2+fmZmZmpp65513AGB0dHR0dHRwcLC5uXnDhg2GL2\/btg3\/x0XE5+g7FApdvnwZPQ0ODuKq2tvbk2xdluXa2tqKigq73Y6HSPLSIuFweGRkpLm52Waz2Wy23bt3v\/LKK4YjPRG9vb1TU1NDQ0OGdsnv9weDQSyt0+l0Op3Hjx\/H8qRZSH04YvScPn26vr7e4XCI+pmamhocHGxsbNyyZYssy11dXcPDw8PDw5DSU21tLQCcOHFiZGTkgQceqKmp+d3vfjc6OopKDGzdujXRerZt2zY6Onry5MmZmZmHH344FAr96le\/EutPRE1NTfIvxGVqampmZiZuCZOAtQ8Avb29sQnU2NgY+siokLW1tTU1NXiI22y2U6dODQ0N4VauXr0aCoWCwWBbW5vD4Th58mQoFHrvvfdCoZDdbo+NvxSetmzZ0tjY+Oqrr2qa9tnPftZutz\/zzDMA0NLSkm4FAIjvv\/rqq42NjXfeeefMzMxzzz2HB05G6zEUbHBwMPb41ddO+mBa5Pf79+7dCwDf\/e539U331q1bFUXRNxLpkCTIKisr7XZ7fX29z+fz+\/1+v\/\/UqVMNDQ2Jvp\/CkyzLra2tExMTJpPp1ltvbW1tBYBV1C9W6\/nz5+12e2NjI\/ZtiRrudBDNguhI+vr6sCfXd6vhcPj555\/\/zGc+kzIJ9Hg8uHhnZ6fT6bTb7RUVFeKvoosFAGyLurq6UhZeluV9+\/aNjo6KDrK\/vx\/bMQBob28PBoN+v1\/ka1jbo6Oj4+PjIlHC+JMSbmSFlpYWWZaxuceWDQMzo9MmLMHw8HB7e7ssy3a7HeK1ky0tLUePHu3s7MSDOjltbW1er9flcg0MDOAmRFLe39\/f19eHycjOnTvTySNcLtc777yDiRmuSq\/BZrP95Cc\/2bNnj8PhAAC3240rT0lTU9NLL70kFgSA+vr6J598UpZlzJswTXc6nZhLY7+IxXA6neLsyDQ\/P5\/O9pi1ha8b0YA90YA90SB1HkGOSCSC\/0uSJEklsoPEdgMd4At8vbS0BADRaBT\/j0QiZrMZAEwmEwBomqaqqiRJmqaZzWZJksxms6ZpQiEVl8VVRBEK4gVWvclkEi\/KysoikcjCwkJVVVV5eTnWssViEfUuPsQ1LC8vG9YZiUQWFxcjkcj8\/HxFRUVZWRkuiEbNZjOaLiqXhcvLYx1gvaMMDAjUgC9MJlN5eTncWF\/4CQCEQqGGhgar1brqwiwuLgaDwU2bNkmSFNelpmmapsFKUAIAxiKKNCjMt8vcrDquA9A1R0KDqqoLCwuVlZUVFRUmk0mSJKxr4UBaIfnmsqwU3ITFYpFlOdGqxL5AgrhcXl42BCVaBICysrJYhdm4TL1YIgf6aMDOQHQMkiRhNOClF72DSCSSZSgUDFG5Sb6Dd2dsNpvVao11iSIxLjEi8bW+v0zTpTQxMRGJRDZu3IirXlxchHgODHFQXl6O3YDeQZLNCLIPhaIC9zdJUELSuNQ0DT9ZXl5WFKWqqqqsrAwbW32+I0mS9OKLL7722mv79u2z2+0iDkRXLAJiYWHh8uXLJOLgN2cHf\/jrp9Z5W7auc1x57ETn7V+94\/bMru7nlmziEl8sLy9LdXV1b7zxhslkuu2229LZXvFzbvb3jTtvL2uZPfbTh2a\/N7fxgfq19ZQOGDNWqzVRGJRdu2xOxIEecS4ViwlM2FyXDCV63SgNR0k0FyFr4ynLOkor+tOLJyoNSYnGEwCkFVTZUrCgLLSnwhy\/pkI4uk4BdqqU46mUuOZpYmJibcuRc0ot39u8efNalyH3XGv3SsgU4XYPz9XjooEGoK168SKERlaaKaoWBdWkpTBFJikHup6SV7GqqqBFC1CMggXl2njK9+5F1aim5nULhWYNPBWgtYmq0ZSNXq4oTONJtd1LTlSNqmqhRBWEa56CweDaliO3RKNRk6alyvgoUbLxZFJTp+aEkACgrq5urYuxGpJcA42q0ZRZOa37GqUbTwVLJArC2njK\/lhOlmVp2O6lTsyzT9UKFpQlGU9aNBoF1VRC3VPpnj+BVqCrsHz+tHqiahTUErpaXgLnT5EVFhYW8O30zEz0lqi2Ek+KooTDYTEcsfjHH8aFUjzplVy5cgUA9HO50YTVapUkKRqNatFr3RMOlsZvzszMiC9PT09bLBar1VpRUVEMMzKSU6SFi40SHFCNYNVLklRTU4NDd\/VRUi3LalRVo9eyCKvVitPrxcBsvAp85cqV2dnZdevW6eWJVeFA4OIJPgkA1vaWrl6Jfow7IsZX22w2fIt1h+N7xW8jGYhGomrEmO1hxIhfG8Ct2O12\/QGxuLioKEokEjHIEyPpDcdEwShcPImB8FevXo3ouF6UBEoyRQNQVVVVU9\/U1c9dMGzLIE+0nPoFrVbrlStXzGbz4uJivuXlxZM+LDBK9Eree+89SZJuuukm7BsgCyXx0VY8ZUESeWLmUyQSmZ2dhZUJRfoOEnKds+TmhDyREnHAiu4aAGRZjkQi2GfkCeyfcn7BPFG6YbPZsNvDfY\/NWfTd3upylgwW0BciHSWJCjQ3N5dpKTPleruXf8QUKP2HhpxlcXERtSWSp++S428l7qexUTI7OxsOh6PRqJixjAGeXMlaoapqNLqWV40MOYsgUc6ytLSE1VtVVRU3Z5GOHDkyMTGhqur09DSuxZBuAYDVasXGatOmTcnnzhWShNdANVCjqnrtOmxxXeNL1O3Nzc0tLS3JsowzDGNzFsnpdB45csRms+HfUElslGAIi06yyFFVVY2maPciRTMDFZulm266qaamRig05CyS0+nctWvX5ORkQ0PD2hY3I5JXsRpNK98rEk9xMXQlJOc\/pURVVTUaLUCjV7D7T4THLSdGU1VVVdUi65uyYg085bu10TRQo1o0WqCBloVpPEsynkDFDorjqchRo5paqHgqDITnqSXpw6\/1T6tdvAgpxXjSQFWjpRlPBSb7Y1n8wFtc1KimqtGU+V7ylRQVZbDW9wlzjrnCfO2n8zQADaQKc\/62VcrnT\/k\/ijVN1fB6RMlkfKXZP6UxPTdnFKbxLEVPABpc+8nQkqE0Pa2EVMJL5rSSchCeiI6zjF\/dK+1eKQVUCcaThj\/DmoaoYr6vYYCqp+RVrBVk0mchG88yWKP5hHncyVSdE0WoxlNyNA1EupdvWSV7X6MQO6aV1CRqKNp5ABkhRrHhELbp6WlNd\/qkzM1NT08D5Uk1QNETWsFZTRUVFZqmiYFs10a+SdL1LgoAVkY34v\/4nbKysunpaf34niKHgCcxpBDDRTyxIBqN\/vGPf6ysrKysrJRlGcfh4rg2gJULEhrI1XJDQwOOkcMRjfPz83Nzc5IkXb58eWZmRoypW8PpGCm57ikYDNbX169hURDDNBt9rACA1Wq12WxYsx\/5yEdwCPTiChgx09MzmnY9NVfmlIsXL+JKcPzhhg0bxGB3MTRVjCgG3WjI4jG3xr9LkI4VSDyhA+c\/4YL4iIr5+fmlpSX9+RN+UlVVpR8\/KtagX61hULHBHDrWmzPMC8orBfUkKmJubu71118HAEmSLBaLxWKpqqrSD8VNcggnV7thw4b169fDSqMHAOvXr9+4cSPWu34Yvhgff30Md8yg4lhz+Pi2paUlfE6MxWLB7+Q7ic3jfEJDGqY\/+m655ZalpSXct+XlZdxbrHFp5alZ+mM2o4CzWqz6tNxqtYpph\/pV4TBukVzAyrB9YS6ie44Sbhq\/Jp4hZjabVVUVjxeP6z5X5Owo0FuBG2fUiuM0UazoG6533303EoksLS3hg+5wekhGAQegP9G94SwqbsTgpjFc8Oly+nABgMrKypqamkSbTu4+V+Yym\/9keBubhsGN0zdTtmAQE3AWi8VsNlut1mg0GhtwGG36gIuPLilPc+tY\/qqqKowV\/aYBQFEUSZIikYjVal1YWEjZWhqmrSWJWrFI8q4uA0+qqr755pv4uDTcAf1cKJGGJWqpcxJwhglf8ZsaDTT1hktH+rqI24RKkpQkXGBlbl3s1uOmhVK8aWui\/Djn6fLly\/gMtKWlJYvFoihK8glLUtyRe3F3CZ+5h18wm824A6Km4u5hlgF3vZQ37rmhe9cfsJIkLS4uAOiuRyhKKBRKlN+n3Dp+Xz+xPja50H\/ZMH3P8Hl5eTnWBtrC1hUALBZLWVnZxYsX9TV5Q9QODw8fOXIEVs7wIzHz1BL1CpFIRBxliqKIA8RkMuFwH2wx0g+49Inb1MzNza04m72W7WkaAMwr87Ozs1VVVTfffHOah8Uqth573ERWnhqorxBRG\/j0Ov1sJ1iZZBY3aiWR7N1zzz2bNm3avn37pk2b7r777tbW1pTJMR4d4kNM4fD0BXsU\/METcaUgywoyFABiehesgl3NX2ra6gjPTEUX1T8rt3381o9jYfDXc+IerVkijj+sE0mSFEWJRqMiHzEctdhx6I9XfdcldjAYDL799tsXLlw4e\/bstefnTkxMvPjiiwAwPDw8PDyMX62vr7\/jjjvw\/0984hOxlQK6hNgQcPpoi9sPZVpHyXsX0dQkWq2+h4gtv7GHS7tIIl\/A35gAXfsRWyGiAPqsQX\/ozM7OTk5OTkxMDA0NBYPBM2fOwMrdwTjPOcYeK1abiLYdO3Z8+tOfzqgNiVtNybXhInEn1qecUp9OefTJdGyR4l4xSrIXKQ+UuGXAcLlw4cK5c+cmJyfPnTsHK2KcTqfT6dy8eTM+mDD186gnJiYuXbqEthJFW0tLZg9qjN1h7FTFQ2bwscHZ1EKmxG0AIpEIPkzc0HatukjBYDAYDGLEnDlzBocPCTFoBcUYyPi54YmiLVNt+kYDr8thgx658dxWluXy8vKCzd8WQby4uIgXBlEVPpYYT0Xw2nyapcLmC\/XEitFHTHKyfb57+tr07VjyRiNR8wIJThKzIc1SGc6i4paqurpakiThAyMGv1ZXV7d58+a6urr0xRjI1pOBJNp27NhRU1Ozffv2u+++G2LyjiSIviR7bYmyRL2YNGNXlOrtt98+e\/bsuXPnLl++fPbsWfxr9mIM5NiTAb22EydOXLp0CT\/Psm+L1Qa6c2G9tiRi9Nl5Ro0qRow+JUPQSq7EGDDNz8+Hw+E9e\/Zg07l3716XywUAHo9nYGAAANxud1tbW\/ZbwvDatWtX9n2bHoM2FLO0tKSqqsViWVxcFNe3Vp1\/xxVjiJjh4eFEKcCq0Xsxzc\/P9\/X1bdu2zeVyBQIBl8v1xBNP1NbWHjhw4MiRI36\/\/\/jx4263O\/anerInJymJIWL0J5h4ERJTkrjRFhesl+Ri0E22+58KvZcb2j1FUXp7e7u6usbGxgYHB91u98LCQk9Pz4EDB5qamvJdrOxTkrhnl+I8yfCjQRhe1dXVk5OTkEAMxkfBxCRCUZQb2uXx8XFFURwOx9jYmN1ul2UZ771OTU0VwBNWRE9PD741aHvhhRfwc0NKkvxyqv5s32azYfCdP38eAF5++eXYzt\/pdPb09OS8BcuS8fHx657C4fCBAwcOHDiAd0jXnETaxCUuJJ1GMlFThhShGD3oRfL5fG1tbYFAADskISkUCmFDUV1dXVtbu7pt5DAZEdomJiYwoaqrq4sbbUKb4UIZrETMd77zHVwDKt+1a1eu2jSfz9fb2wu6dCxT+vr6Tp48CSs1JryY5ufnA4HA008\/3d\/fL5IF8eds8oicrCRNEvVtec2VDYTDYezLAcBw0K8OvRdJUZRDhw4NDw+LRgNNdnR0tLW1ybLs8XhWV79DQ0N448rhcCiKMj4+nr9OLm4jWeCe3+\/3z83N1dbWVlRUyLLs9\/uzaUIMXiRZlo8dOxb7PZfLtbrI1VP4ZARZq9zMbrfjL0gDwNjYWDaeDF5Kc15N6ZFfT6tIRsLhcEdHh8PhcDgcHo8HP\/R4PPiJz+fLeSEVRenu7sb19\/X1GYrR3d2tP\/FKTigUEje4t27dmsNC5tFTS0uLoigLCwt+v1+W5S1btqSz1MDAwO7du\/1+v9frPXr0qM\/nCwQCp06d8vl8brf7+PHj6ddamni9Xrvd7vf7fT7fyMgIHhxYjKGhIQAQWUlyHA5HdXX11NSUOA3NYSHzeFOnqalpFclIf38\/vtiyZUtjYyPkPx8R3bDNZmtubgaAcDg8MjLS3t4uy3Jra+vp06fT6WlsNltPT09nZycAuN3u3J6G5vfmWzbJSOEvjoTD4bfeeuvBBx+EGxtqbL3TOc7a2tr8fn8+ylakeUThL44oirJ\/\/\/6enp6CJaUZUYyeAoFAT0\/PkSNHRJXl5OJIEsLhcHd39759+0T7hoGLrzGac77RjCg6T3gSfuzYMRFJq8tH0iccDu\/fv19\/WGBHNTY2pijK4OBge3t7bre4CvJ7PzdT8MaKPr\/CiyN4nRDzkZy3S+KSGoKX5sQ9up07d4rUZg3JwBMWfffu3SI1iP2ESYnP5\/vWt74lDjhxbzZ5SplZPAUCgUceeeTw4cO4DTwrLIbDjRYejwdvwwJAb29va2trygM9s\/6pqanp\/vvvP3TokKIogUDgrbfe2rt37+rL+2EFz7G8Xq\/X6xVvk5Nx\/4Rt3Te+8Y0XXnihq6srJ0NcPoRgcwcAafa4Ged7Npvt0UcffeyxxwCgaO+BFj94taWxsTHN9HU1eTneeevq6lrzswq6eL1eRVEURcGmLyUEfo+l9AgEAj\/\/+c8PHz4MAI888khLS0vKpq\/oznNLnnA4\/M1vfvP+++9vamrS52XJl2JPhWb\/\/v12u13kePgi5blNcV2PYBLB8UQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kSDdD398r\/N9qpK\/b+nDpfntWSMHtP8\/Hymyzx1uPzHz0j\/9evFTZu0fJSJiUXKdIFzvy879rT0o58tsaRCkln\/NDlpcj1g6f5aZPtfqnkqEBOXDNo9RYE9f2Otb1DdRz\/Ia5mYWFK3e799rewfvmYBgO071IsXTD\/88XL+S8UYSe3p4njZxXGTBvBuyHzyFc4d1oYM+qcPPoCOVqvIy3sfWpe\/YjEG0s33TADff\/qDzq9G8loaJhF8PYIG7IkG7IkG5scff1z\/fuHoD9\/\/Wrd02+3q1NTyq7+e\/fu\/tZ\/5N2lh9rV7nvt91dEZuPjaxSGTCQBMNdabAKDzF3\/3H+dP7m7cCQDvfzD3V8\/+9YWZP7Xd2roW+1LKxDnPnfnylyJn\/w9MZQCaBmDSAEymLQ8Z0\/FTX\/Z+quFO7+gvHvf1+x480bB+82+CZ75+6tGXdj\/bsH5zocr\/YSFOu1f58NfBBJqmAZhMYAKTqWxzfaLl76r7JAD89tLrAPDLd3wf3XAbS8oHcfLydZ\/5K8nxFxG\/3\/K5e61f6Sqrrzdvrp8HGJ8N\/un9i+OzwfHZ4J\/eD36q4U4AaFi\/eYd9e2DqTWiEN6+8fd\/H7i34LnwoiH\/+ZPnc5yN+P1gs5Xe0iA+3rK\/fsr7+Uw3GL9\/3sXufGvYEwucvzU1ieDE5J067F\/G\/sfizn8r\/dPCDV04v\/e\/\/pFzFXXWfnPtA+dc\/\/Htd9SZu9PKEMZ40RVGePGj98lesD3xVU+auPvFtyfHn5o0bk6yiYf3mj2647Zk\/HP\/R5\/8ln0X9UGOMJ+Xb\/6hdurRu530AYH1gj6mu7ur3\/jnlWu772L32qo3c6OUPYzxVPzkgXptkueb4z9Jc0Q77dm708kdurkf85\/mXOdPLK9l6+k3wzMbvNwJAZ+OXclEeJj6rGW\/EFB6+DksD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kQD9kSD\/wfT4q3y2m\/6pQAAAABJRU5ErkJggg==","height":388,"width":141}}
%---
%[output:096fe590]
%   data: {"dataType":"warning","outputData":{"text":"Warning: An error occurred while drawing the scene: SceneModel error in command compositeCommand: TypeError: Cannot read properties of undefined (reading 'deferred')\n    at https:\/\/matlab-1b.mathworks.com\/toolbox\/matlab\/uitools\/figurelibjs\/release\/bundle.mwBundle.gbtfigure-lib.js?mre=https"}}
%---
%[output:59f0065e]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAGkCAIAAACgjIjwAAAAB3RJTUUH6gQaESsZvtAfkQAAIABJREFUeJzt3XF4G+d9J\/gfxKEJSCMLsgNaIKEwldonVcg8uZPLhKnVXuHn\/JyeJqbSPstHm3qlVsc6adUy97D20Qnje55sm8gr1xfuPdxVN9Vi1YeKk6pMthb97J667gq+rdLSRa3d3pFRdjdWTRsQZCK2KXEogOYAc3+8xMvhABgMgAHmncH3k+dxgBEIzACD+eJ939+841tbWyMAAACn7XB6BQAAAIgQSAAAIAgEEgAACAGBBAAAQkAgAQCAEBBIAAAgBAQSAAAIAYEEAABCQCABAIAQEEgAACAEBBIAAAgBgQQAAEJAIAEAgBAQSAAAIAQEEgAACAGBBAAAQkAgAQCAEBBIAAAgBAQSAAAIAYEEAABCQCABAIAQEEgAACAEBBIAAAgBgQQAAEJAIAEAgBAQSAAAIAQEEgAACAGBBAAAQkAgAQCAEBBIAAAgBAQSAAAIAYEEAABCQCABAIAQEEgAACAEBBIAAAgBgQQAAEJAIAEAgBAQSAAAIAQEEgAACAGBBAAAQkAgAQCAEBBIAAAgBAQSAAAIAYEEAABCQCABAIAQEEgAACAEBBIAAAgBgQQAAEJAIAEAgBAQSAAAIAQEEgAACAGBBAAAQkAgAQCAEBBIAAAgBAQSAAAIAYEEAABCQCABAIAQEEgAACAEBBIAAAgBgQQAAEJAIAEAgBAQSB6xuLg4NDQ0sN3k5CR\/QCaTOXr06NGjRzOZjPlTxWKxgYGBWCxW0wpMTk4ODAzE43HDcva6A+VUfQm2UadOnVIUpaaVqcpkG2t9UcM7b\/F907+K4aOp9E7qWf80a2XYnGa8hO2a925AiyGQvGxubq7xo7miKF\/60pcWFxftWisB1b2NsVjs+PHj+nd4amqqGQnaGqWbk0wmo9GoeTq2xuTkpAirAU0lOb0CYKfBwcHp6WlZlolocXFxdHQ0kUgkEoloNBoKha5cuWLlSUZHR0dHR9ntTCZz4sSJlZWVL37xi\/Wtkv51Jycn5+bmpqeno9Golb\/t7++fn5+v73XN2bKNi4uL58+fJyK+RbFYbGpqir\/n9a3bmTNnzpw5Y\/4Y65+mdfF4fGpqinSbw96ZZDI5MzMzODjI9qvWUxRlbGwskUg89thjZR\/QjHcDHIEWkmf19\/c\/99xzRPTKK69QSbeGoiinTp0aGBgYGhpKJBKnTp0aGhpiTQTencWPR4qiHD9+nHUAxuNxfbdb3T9a9R1TvFuP3eXdWX\/3d39X2q8Vj8d5n1Lpq7Pt4tvCnorfZSs\/OTlpvo3MjRs32AtV6guan59XFGV8fJxnz+jo6PDwsH6JvseSr4b1d4bfZXjbq7STSt\/Vpl\/Ot5Q\/T6UGHNtP9CsfCoWeeeaZSCTy\/PPPszTiu03p21JpS9l7furUqS996Uu8V1O\/++nfE\/3exTo\/eRoR0djY2KlTp5aWlti2f+Mb32CrcePGDcO7wba6dCfRP7\/FjwNaCYHkZd3d3bIsp9NpwwFI\/yVnt2\/dumXlCRcXF7\/yla\/ol5w9e7a+jnv2a5cdBBcWFpLJJBHdvHmTiJaXlxVFCYfDgUDA8FfJZHJsbIxvTumry7J85MgRRVFY04plhqIoy8vL\/PkPHjxYdfVu3brFXyiZTE5MTJQexN944w0iOnDggH7hmTNneNtrcXHx8ccfZ5tGRCzzaopw1qbkdxOJRNnGUzwe13e1JZPJxx9\/XH+0nZqa4s+TSCQuXbpkeAZFUdLptCzLQ0ND+uXRaPTKlSuhUIi27zbsVU6cOMHe\/6pbmkgkrl69yleG\/Qxij3zhhRfYmsfj8bGxMf06mwzIJZPJ7373u0QUDod37typ\/yfWTuV3x8bG2JoYnl9RlKeeegrDTkJBIHlfOp3OZrP6JUtLSzdu3JBl+dKlSwsLC88991wqlSr9w1AodPHixUgkwh555swZ1oe2sLCwsLAQj8cjkcjKygo71tdKH5Yslojo2rVr\/G6l\/pnx8fGFhYVLly7Jslz21VlCvPHGG4qiXLt2jS185ZVX2F3DMbd0G9nyVCr13HPP8RcqfQ\/ZEdx8G1988UVFUQYHB9mbNj4+TkQzMzO1jjBNT08vFJUGkqIoMzMzRDQ8PMweMzw8rCjKiy++yB8TiUTi8Tj7JypGqV42mzVsjr5lxhofrCtSvznJZJLlXNUtZW\/v\/Pz84OAgEbFH6t9bvhVsY9neNTs7m81mp6en2V9NT09fuHCBxw\/bE\/RLiCiTyczOzvJ9m72Efk34u7SwsMCzFgSBQGpHrAny6KOP9vf3E9Hg4CD7wlvEOmei0Sj\/RVyHvr6+Q4cO3bhx48aNG+l0OhKJDA4OptPpt9566\/r165FIZGBgoPSveJywPy\/7zAMDA5FI5Pr162+99VY6nT506NChQ4fS6TR7rUOHDvX19VVdPb4C3d3dwWCwjg3kbY6nn36a9XcNDw9HIpEbN24sLS1ZfBKWymNjYyYdpCxLIpEIiwEieuKJJ2RZvn79Ov\/5f\/jwYXbkrRTzgUAgHA6brwxrXyYSCdY3yFohLPirbil722VZZq9y5MgRWZb17y1PRLaxbO8y+blT2phjlpeXV1ZWWBNtYGCANRxZ5rEdY25ujr2Z+u5ZEAQCyctM+r7qwwYqJiYmvve977HfsHU\/Fe9bu3jx4o0bNw4fPnzkyJFkMvntb387mUzyA6hBMBjs7u42f+ZQKHT48GH+VEePHv2Zn\/mZGzduXLx4UVEUdiisunpV3zS2\/lTsdeTYAFXV57coGo0uLCxMT0+zu2wQpY4Svqq9lCwqeFcnEZ05c4a3VGp9uaoM\/Zx1sLInGLDaB75Fc3NzGEYSDQLJszKZzNmzZ4no5MmThkMw6y67evUq+zayrpiyT2L44cx6Ztgxnf0UbWQNh4aG2GooivLYY4+xu6wLqNIPeYvYn8\/NzbHf0Y899piiKFevXi37s9pK48Bk\/efm5njDJR6Pz83NTU1NxeNxfojnYyRzc3PJZNJiE02PxRI7kpY2sNj6J5NJPnDCPqZKoV4Je9MMIzdTU1O8HcxShPfL8S5EW7aUfwr6\/sn5+fn+\/n7errKCtbp4l11p1xyLJd6x2aQyTqgPyr49hXWn6JeU7Y5j\/V2JROL48eNEJMtyb2\/vnTt3Kj0t6wAZHh5mP7Snpqb4sa+RUmC+Gqz3ht+t1F9nHeucSSaT7Hd0d3c3u2tyiOTb+MQTT1h8lf7+\/ieffHJqako\/VE669\/yJJ564evWq4UMp\/X1QiaGIgD95X1+ffkBLluWTJ08mEom5uTleuSDLsvUNYaLR6Pj4+FSR\/p9YtrHtMmwOqxFvcEv1W6F\/M\/WnMRDR2NjY4ODg1772NZPnCYVCIyMjU1NTbN9mhoeHz5w5Yyh2oMr9fuAUtJC8bHx8\/MKFC6UHBVmW+UAxu93T01P2GXjHFBGl0+nHH3+cx9v4+Pj4+HgjvzH5kxsGGGr9aV+K9drxp+K\/vsv21xm20VC8YG50dJQNm\/Ml+ve8v7\/\/5Zdf5l1e7Ge79fOTZFm+cOECK0Ng9BXYetFoVL8akUjk5ZdfZgOENRkdHTX00bFqCFZJod9tGH7GUoNbyreCd07S9jTiLeZ0On3v3r2qW8GH06iYRmy5\/vmJ6LnnnqvjXYLm8a2trTm9DtBq7JxZRVHYAYXdJaJYLIbvJwA4BV127Yh3jhm6R2od2wAAsBFaSG2Kz1DA7ho66wEAWg+BBAAAQkCXHUCzqKrK\/itJkiThuwZQBb4kAHXieWO4kc\/nVVXN5\/M+n4+I2H+JqKOjQ9O0jo6Ojo4Onk+SjjObASAMfAcAyrOeNzt27PD5fJIk+Xy+zs5ONsWDJEmdnZ08ZlRV3djY4M\/D72qapmlaoVBgtzs6Oohox44dPKKQWNA+sItDmyqbN6qq+nw+fd6whDDPm0bSQv\/q+sQiInZGVNnE0jez+Kv7\/f5G3xQAR6GoAbzJkDe5XI6I8vk8+y9z7949WZZ50rDUYQd3njrUWN7YuyGGxNrY2FBVlSXWxsbG2trazp0777vvPqqQWGhmgeCwd4IrVc0bKnamsRssb\/x+P8+bbDb77rvv7tu3j2WSg9tijmdJpQewd2B1dTWVSj3wwANdXV2liUVEvJlVOpSFxAJBYOcD4fCw4f\/V5836+nqlwRvWh8Y606h4EK\/UkSVJkqIoPKLci0fsrl27gsFg6faWtrHIdCgLxRfgFOxb0GoWiwUqDd4Eg0GeNzg+WlG1jUWVOwZRfAGthF0HbFY2b3ilALut70yTJIkd1\/TFAoS8aS2LHYMovoCmwrcdalM2b6oO3kiSxA5DdhWnQYs1klj64guWWIShLCgHnzpsU5o36+vrd+\/e3bVrFxVHcfjgDREZigWQN22rkcRiS8oWXxBRLpfbs2cPyy3sWt6GD7W9WClO44M3RNTZ2cky6cEHH+zq6hKnGBpcp47EIqK1tbX33nvP7\/ezNndp8QXv8uWvgs5e98Jn5il15I2hGJpK8iaXy0mS9KEPfQhd\/9BUZRMrEAhsbGyEw2G2+9VXfFHaMYjEEhM+EtcwFEPriwV8Pt\/6+jpVGLwpLYau6duI7y04Tt8AIvuKL4iIfV9QfCEIHGtEYV6cVrVYYM+ePShOA2jSUBaKL1pDiHdTUZSxsbEjR46wC2kTUSwWm5qaIqJIJHLx4sVQKOToCtrAet7oO9MMxWmEvAFoTOOJRdWGspBYdRPizbp06VIikThy5Ai7G4\/HZ2dn4\/F4KBSKxWITExPiX8zUSjF06eCNhGJoAMHUV3xhMpTFE4s1s\/ir4MdlKeffi8XFxWvXrh06dIjdVRRlZmZmZGSEtYqGh4evXLmytLTU39\/v4EqW5k0+n19fX793715XVxdt70wjovvuu69qsQAAuFHVxKLaiy+IaG1t7f7777\/vvvvaufjC4U1VFOVb3\/rW6dOnz507x5Zks9l0On3gwAF2NxAIyLI8Pz\/f1EAqLU7jxQJ8cgGeN2z8hogkSdrY2HjggQeCwSAhbwCgqNZm1vr6uqIorKekbPGFyVCWl4ovHD50JhIJWZZ584jZvXt3d3c3uy3LcjgcLv3DVCrV29tr8VUs5o2hM41\/3vr2jf6zV1VV0zTUQwNArQyJlcvlstlsJBLhS+obynJ78YWTa5nJZGZmZp5\/\/vk6\/vby5cvnzp3r6ekZHBzs7e39zGc+89BDDxmKoa0P3hCKBcBWP\/jPP9jR6fv0wM\/\/+r\/6td957H\/75MFP\/T+3X\/nYvo+FqFelD\/46\/YNfDEedXkcQWsuKL4Q67jm5HlNTUydPngyFQoqi6Jevrq4uLy+zPjpFUdLp9MGDBw1\/e+zYMXYjkUhcvnz5r\/7qryYmJvbt24e8AQf9z2d+ceGVHxY2NGm3JO3u6Lxf6twtBc999t\/uunX4ovJT9JF3KfXtH377L\/\/bX37hP43uVR78mz9OOL3K4FZ2FV\/s3Llzz549LVnl6hw7QGcymevXr8\/NzfEliUTijTfemJycDIfDN2\/ejEajRJTNZhVFGRoaMvx5b2\/v6dOn2e2XXnrp2Wef1TStNLcAWkmSpQ8f7+kKdP3mr\/\/mEYo+uBF64Udnrn8s9vabye\/9YS5\/L5\/P5td\/8kE+W9DyWiAsdOEouJ3F4gt+6pUIHAukUCh05coVdttwHtLJkyfPnj07PDwcCoXm5uZkWe7r6zN5qsHBQcKEAiAOjTTSNm8I81UHKMUqs5xeiy0iHsSj0ShvIbETYwU\/CQnAgAWStnkbACwRIpBkWb5w4YJ+yejoKJ+1AcB1NNI2M2nz\/wGguh1OrwCAhyGLAGqAQAKwnaZpGvEWEmIJwBoEEoDNtK0cKji9LgBu4p1ASqVSjrwuK\/YHMNA2gwlldlAdDiOMdwIJQBzFFhLCCEQn1E8mLwSS9Unt7IUzn6AS3mW3WWUn0FceQFxeCCQA0WikaVoBFd9QE\/zGRSCBJejjro22eSqSSZVdh9zR4pVyI+x4bQWBBGC\/YhChyg6gBggkAPttzvqPKjuAWiCQAJqCzx4EABYhkKAKDLTWQStW12maRpisAcAa7wRSMpl0ehUAiGizxk4jrYA2kk3wq6hNeCeQAMRRPAWpwNpHAGISbYjTI4HU09Pj9CoA6BTTCC0kAOs8EkgAQmE\/PAusrgGZBNUIddlWByGQGoU9CbYUs0fTtAJpGoskALAGgQTQFGzqIHZNJKfXBcAdEEgNQfEPlKOxFlKBcF4sWIWDCSGQAGzHL9DHpg7CIBKARd4JJJyHBELZnM4OLSQAy7wTSADi2CpqYHedXRsAl0AgAdhP42fFooUEYJlHhtGcumgsALtgj6qq+iv3sHGjAmsaGSJJ2\/rDXC4nSRJGs8EphYJYV0jBNwGgOn3qbGxs8PjJ5XL6h2n38RORilMHmbaQ0uk0u8EySSrq7OxkS\/x+v\/0bAyAqBBLAJoupow8PWZYDgQBf0ilJlCfaHEMqFKhAladqkCRp\/\/79hpdTVVVRlNJHckQUCAQIWQVehECC9sKP+7lc7r333uvs7Ozo6DB0uNH21AkGg7U1WTbHjgoFrcrkqpX66wzRSES5XI5n1crKiuEZJEnK5\/N3796VZdnkaQEEh70WvMYwqJPNZkmXQ\/xh6+vr6+vrqqp2dXVJksSaHfqGSCMKhUJBKxRYl13tZQ08Dsv+K98WllW8XXX37t2uri7ewOJPwkKUZSqyCkSGXbNRhl\/W0Bp1dK8ZUodF0b59+5rR96VpWqF4CXPbq77LhgqrjwiFQn6\/X1VVfQwrimLS\/kM3IIjDO4GEE2O9p\/FBHZPWQFN\/SRSnDmpSJFXEM6Y0WuroBmTvHppWLYDftQz2MHCYvjOtUvcaNTio02pasaghvzWCpOn+64T6ugErPQm6AaEZPLIb9fT03L59u\/Wvi++hFRYHdWj7b3N7B3VaR6PNq5drhYKvStm3OEzeZN4qNWQVugHt5aadvGnwFoAlVS\/71Pigjge+kFvNoWLZt6a5\/iLm7HNh9XsGZT90K92A1rMK1xtrK64\/BEArNXVQx0tYUUPeV6Xs2+1MugExZAV1wEcORqWDOplM5t69e\/qDCLlsUKfVCpuTq2rUwooGodQxZMUSy\/Akd+7cuXfvXjAYJGRVE4jWpYzPtR3VOqjT0dHh9\/tDoRB5qHutqTStUNAKBS2\/rcZOrO++k6oOWanFyvX19fV79+5hyKpN4LDiWTYO6uRyuWAwWHYUAcrQSCvoxpAqPwxKSdsL+Yioq6tr\/\/79hG7ANoCPx91aM6iDr7F1m9XdmqZphTwVitefQPjYwK5uQELluqi88xmkUimnV6FZSgd1KnWvEQZ1nKdRaZUd8qj5rHcDonJdWN4JJFerdVDHpHsNHMYaRHymBjSOBFDaDcg12A2IrLKXR45ivb29r7\/+utNrUYWNgzogMo00raAVtILmifOQvK2mbkB9VpU+id\/vx5BVgxx+v+Lx+NjYGBFFIpGLFy+yOi4iisViU1NTpcvF15pBHRBcQdM0bauowTB\/ELiFeTdgTUNWTKWOdMNftS0nj33xePwrX\/nKpUuX+vv7Y7HYxMTE9PS0LMvxeHx2djYej4dCIf1yB1fVgO+L9+7dw6AObClOWMfSKF\/v5SdAfLUOWZln1d27d4lo79691N7HBycD6ZVXXnn00Uf7+\/uJaGho6M\/+7M+Wlpb6+vpmZmZGRkZYq2h4ePjKlStLS0vsYS1jcVDn3XfflSRJVVV0r4Gepmn5aleMBa+qY8jqzp07pLukPbWqch0nxm45c+YMvz0\/P9\/T09PX15fNZtPp9IEDB9jyQCAgy\/L8\/HwzAqnxQR1ZllVVDYfDtq8buBor+2Ynxor1jQdHmQxZSZK0d+\/emroBvVe57vwGsGEkWZZjsZgsy9lsdvfu3d3d3exfZVkue7hPpVK9vb1Wnh+DOtB6xRNjNX2XHZIJzNXaDei9ynXnD7XRaHRhYSGTyZw4ceKZZ54ZGBiw8leXL18+d+5cT09Pb2\/v4OBgIpFIJpOs+gWDOuCczdDRtM1LIqG\/DmxRRzeglcr1jo6Olm2CFc4HEhMKhQ4fPvzKK68MDAysrq4uLy+zPjpFUdLp9MGDBw2PP3bsGLuRSCTOnTvHbmcyGcKZOuAoNmbEx5Baer1YaEsm3YBU7dKLu3btEqpezLFjtKIoY2NjR44cGR0d1S8PBALhcPjmzZvRaJSIstmsoihDQ0OGP+\/t7T19+jS\/+9JLLz377LP79+9H6oDD2ATfxbJvdr2+4pzfSCZoNfNuQE3ThLri1A6nXliW5ZMnT87OzrJmTTwev3r16hNPPGFYPjc3J8tyX1+fledEGoEYtMLWJczRQgJBCXjAdHKFotEobwmxogbWTadfzk6MFapRCVCVVigU+ImxyCMAaxxOyNHRUUOXnflyAFcoTq6qoewbwDrHuuwAPGhrpobNyVV1LSQEE0AVCCQA+22dGEuEKAJhidaARyA1CrMiQil+PSThvvEgJBxGGE8FUjKZdHoVAIhYIBUKhQKKGgBq4KlAgubBL7iaaAU+kx3iCCwRsAi79TwSSBbntbMd9iEw0kgjKrBL9OWLMzUgluqFX0JtxSOBBCAQVmWXLxQKeQ2XnwCwDIEEYBs+rzcfQyJDIiGbACpDIAHYT9MKrI2EogYA6xBIAPbTClohrxUKBY2QSABWIZCgOtRuWFWcSVXTtIKW1woFlNk1Drtf8xQKBadXYRsEEoDdWFFDQTOeh4RoAjDlqUDCibEgCFbUkM8X0D4CsM5TgQQgiG1VdmgZAVjjkUDq6elx6qVx4h7osfApFDYTaWuuBqQSQDUeCSQAIRQn9942lx0RTo4Fc\/hdyyCQAGzFq+z0gQQAFiCQGtLZ2en0KoB4NidqYFV2ZJjIDgEFUAkCCcB+mqYVtEKhUDDGEUAF+HVLRDjjDMAGm2MAfDK7gqblC4UdeZOpgzBsAGDgqUBKpVJOrwJ4HEuRXC5HRNlsVn+XiFR1g93QNC2v8TGkim2kf\/iHf2DTEEiS5Pf7iaizs5PfBmg3ngokALsYgoct4cHDSEXBYJAFiSR1EmspbY4iaSaX6JMkKRQKbWxssBdSFEXfZjIEVSAQICIEFdhLtKIbjwSSUxfoA7dTdWoJHslsgjU+dVC1mRpkWa60PoagWllZ4StDCCrwKI8EEoA51vjQB8\/a2lo6nV5fX+\/q6mKP4Ukjy3IgEKgePGVpGrErxhb0XXZWf4dWekV9ULEbhqDK5\/PLy8s+n2\/Pnj2EoAJ3QiCBp+iDhx+7Gf4YnjRdXV0f+tCHWDPFxmO37sTY\/GafSMP9IlWDSlGUO3fuEBGLKEOLyu\/3sxsIKhAZAgncqrTRUBo8VDwWsxYP6Y7FuVxuY2NDlmV7o2jzRkErFPgF+prYT8+DSpKkbDa7b98+tjllW1RkGlT1NAcBbIX9D0SnTxpW2FZT8LTWZvRsVjOwC\/Q5MWxs3qLio2UIKhAK9jMbqKqKb2zjLAYPG89nXU\/CVUhvzqOq8Uja1lknQEETS5fSN81KUOkH1RBUNlJxRloR9idotdLgId2pPOSK4KmGddS56OrlZYOKD8ghqKA1PLXf4AJ9QlFNzyElXfCwM3JId3RzZo1tUWwkFVtI2rblrmIoMecMQZXL5cyDylKtfAVoPbQVN3\/zoYVMjgtqtXNI9cFj9VQel+N9dk6vSFPUEVT6vzUElbvavs3jyNdBtF3UO0cER67R5+FDaim15FQetfFzSL2q2GdnT9G3SyCooEHtd6SAakqDJ5PJ5HI5Q6OHaegcUq\/StvJo89p8W\/XgTq6XU8yDitXfU4Wgeu+99yRM9Nc2cARpX2VbPKWFbZIk7dixY9euXaFQCMFTxVbeFE9Ack9dQ+uxHans\/ElUDKoPPvhgfX3dMNEfba\/CQFB5Bo4sbaH0NMmaTuWRJElV1dIDB5Tihd+apmkFJFI99EHF2uXhcJjKtajMZ6RFULkOAslTGgwesIHGL2LuqrpvgfEWuUmLiu\/2hKnT3QyB5EoWzyHlwUMuPJXHxbRih91mDRNCqbkqdSNXCipMnS4sTwWS985DaodzSD1JK9Y1II0cZCWo+Nm+CCoReCqQ3Eut6xxSfEMEttVlpyt0cHB9YIv1oMLU6S2GQGqpmoKn3U\/lcbFijx0VT0NCFLlBTS0qwoy0TeCdt8zBi8aqJZOrlq2oxjmkbcHK1EHIJ1cxD6rGp05XHZoeSbRpGkiEQIrFYlNTU0Qky3IsFuvv7zcsj0QiFy9eZP1UouF7JIIHtmibUwdRYfMueJL+XCg9K0GlPw74fD6nMkk0Dh8WY7HY7OxsPB4PhULxePypp55i2ROPx\/nyWCw2MTExPT3t4HkwZVs8qqqura1lMpl8Pt\/V1cX3MExe0N40It\/2qYOgvZQNKnX7\/En6oFpfX2eHkV27dumPG23Y9efkpiqKcu3atZGREdb6GRwcDIfDCwsLg4ODMzMzfPnw8PCVK1eWlpZ446l59MFT9QLYLHuIaN++fcFgsNnrBi7A++e2V9lp+kcgotqSoXKP430qDz74oCRJTZ06XXyNblImk5mYmHj++efLdqnF4\/GzZ89W6nCTZfnChQv8bjabTafT\/MaBAwfY8kAgIMvy\/Py8vYFUOlBZGjxU7RxSVoeDMhtg9MGjbbWQUNUAFbFo6erq2rt3rz5j1LackdaGjE2n09FodHh4+MyZM408z9zcXDgcHhwczGazu3fv7u7uZstlWWYThxikUikrhQz6pME5pNB0mwNItFlnR9q25hGANSYtKvJuUNnT6PvCF77wne98Z2hoSF+VUJNYLHb+\/PlYLCbLMnujq7p8+fK5c+d6enp6e3sHBwdTqVShUGAfjFsvgA2uV5yZocxZSAA2MA8q86nT9YNbnZ2dXV1drVtva+wJpI9\/\/OPz8\/OxWOz48eODg4O1FiDwNOJhtrq6ury8zO4qipJOpw8ePGj4q2PHjrEbiUTi3LlzRBQKhTJciRdOAAAgAElEQVSZDIIHHLcZSZvnIWnGgSQAu0kWpk6n7RP97dmzR7QDo53DYqOjo8ePHx8bGxsaGpqeno5Go1b+anJy8vr16y+\/\/DIfZwoEAuFw+ObNm+wZstmsoihDQ0OGP+zt7T19+jS\/+9WvfvX27dv79+\/35FgfuAyb6tunEcrswFEmQdXR0WGxO6pldtj7dKxOYXp6emxs7OjRo8vLy+aPj8Vi169fN1Q9yLJ88uTJ2dnZTCZDRHNzc7Is9\/X1WVkBpBE4i19+YnOKVSp\/aiyAs8Q8VDZlnaLR6MLCwuTk5B\/8wR9EIpFKD8tkMrOzs8lkUt+WGh8fHx0djUajvIXETozFxXjABTT+X03TNM1Xch4SQgmEUSgUnF4Fo0YDKRQKXblypew\/nTlzZnx8fGJioo6\/JaLR0dHR0dEGVw+g1TbTaKvKbnMxogigmua22kKhkP5MI4B2wWeywxR2AJaJ2I3oOpiHCgyKzaSqDwLAAWQLAgkswXemNnyOhu3XjEUGAZiwucrOWalUqsWvKGalCjiMTdZQ0LTC9jI7qB1+CbUVTwUSgMO0rWtPaFTAJczBOvy6JS8FkoMX6AMw0jB5EEDNkMkA9tN45TehjQRgFQIJoAk2z0LC1EE2QF9W+\/BOlx00D44ItdB4f10BF4wFgQm4e+JAA2CrrYvGahoVMM83gHUIJAD76NOIddlR8SoUSCWAajwVSMlk0ulVAKCtXjsRO0UAxOWpQAJwVnEiVd3\/UPcNYJl3ihp6enqcXgUAXSYVi+yQR2COXcsVCC0kW2B\/Atqc5IZfbIKPIW0ucGqtAFwEgQRQD7Uom82qOryLTnceki6NtG3P8Pbbb0uS5Pf7iSgQCEiShAp7aGfY+xuFI4jn6bOH3c3lcvxfWYr4\/X6WKJQn4i0k05oGSZJkWc7lcoqiqKq6srJieEIiCgQC7AZ4Gw4jDN4FgC1sbmmWN6zpU5o9kiSFQiF215gWukuY82Gk7cu32bt3r\/6leeblcjmWT+y\/7Gjl9\/slSers7CzzugC1E7AGFIEEbUrf4abvdmP\/yjNAlmXW9LGSAb17I++8c5uItK00slrVoG8Y8TUkolwut7GxwdZNURTD45lAIKAPTgCXQiBBWzAM+RDRnTt3MplMPp\/v6uoydLvZMJaj6YaQ6sXWQZblshuysbGRy+VYSq2srKyvr7PN2bVrFwalwKW8trMmk8lIJOL0WoCTShsWZbvdHnzwQSLat29fMBi08+WLV4jdaiHZWmJXNmNUVV1dXV1fX5dlWdM0fXcfYVAK3MNrgQRtpXTIp2y3WzAYLB16YdUEzTo661pIWoUBJBuxXrv7778\/GAzyLcKgFLiOdwIJF+jzvLKV1uyf9EdY27rd6qBtto80PlGDc+oblEJ3HzgIOxyIqKZKayEPnRoV+LyqojAZlOINKdSgg4NE+xpD22m00lpEmqabzI5I6IkaShtSZNrdR8VPgcUbUgpshECClrLS7VZTpbVQ8rnC5i1N03waaT6Ro8iE9e4+DEqBjRBINuCHVNBzf7dbzVLvp8i3eerRZiRtH0nSClt39oc\/7MAq1qtSdx9hUArsg\/0DLDEPXS92u9VH00jz0Y5CodApdW6oaofU8UH+A9LIJ\/m0AhHRDsmXL2hElM\/mHV7ZhrVgUKodfu21wzZahECCmnm7260hGpGveFM32bfWTlehqHVQCt19TsHUQU3X+hNjvd0LwY4ja2tr7733nizL7dDtZgdcdcKovhp0SZLu3r3LJkbydkrhi8PgXYBNJt1ud+\/eXV9fV1W1PbrdGqVptFXtLd6PUEGYdPepuomR7t69S0SsaYVBKc\/zzseJK8ZaV5o95t1uwWBwZWUlHA47uM6uoRH5mjFnUFsozZhcLifL8u7du01q0HGmlGd4J5CgEsOQDxWjiCxPcIBBV5sgoOrBYgaDUu0AgeQp+h4PdqNStRv\/0jq3sh5XfpIGRJJN6huUQnef4PCRuFXVSmuqPK8oNN1mf52mm6lB4xOBQzO0oAYdmg2B5A6otHaFrfjxFW9o1i\/RB\/artQadMDGSoxBIwmnDCQ48RXcqEl8AQrHe3YdBqRbz2rEslUo5vQo1wAQHHmYYQUL5t8gqdfeRpwelCoVC9Qe1lsveQVdDt5v3aaT5yEf6AaTmX54PmgODUq3nnUAS6gJ96HZrY9umD0ISeUytg1JWuvtwWgUn0HEwHo\/PzMxMT0\/znySxWGxqaoqIIpHIxYsXWc+VaPL5PG\/Lo9sNiF2+3KePIoSSx9VXg84EAoH19fWuri4H1ls8ogRSPB4fGxsbHBzUL5mdnY3H46FQKBaLTUxM6LPKEWwnM3S7pdNpthDdbkBkLGoQ\/wJ90Awm3X2qbmIkRVFWVlYymQwVjyGuHpRqnPMbnMlkTpw4sbKy8uijj66urrKFiqLMzMyMjIywhsXw8PCVK1eWlpb6+\/tbtmKGIR8qmeCA7T3d3d1EFIlE2nDvARNoIIFB2YxRVdXn86mqKssyJkYS4hj6u7\/7u5\/97Gdjsdi1a9fYkmw2m06nDxw4wO4GAgFZlufn55sUSKXt65omOODNI4BN7Ap9mM4OqpEkqaura9euXXv37uULGxyUci\/nD6OhUOizn\/1s6fLdu3ezxgcRybJs18yemOAAAARX36CUB7r7XLnSXCqVMi+uQ6U1tBQbQCo5N9b4GIAatUkNuriBtLq6ury8zProFEVJp9MHDx40POby5cvnzp3r6ekZHBxkyfTjH\/+YjxCi0hocYJg3CPkDTVNrDTptnxips7Nzx44dLV9rM4IelAOBQDgcvnnzZjQaJaJsNqsoytDQkOFhx44dYzcSicTly5eJKJfL5XI5VFqDwzTS2PmxhovHIp6gyax39\/n9\/j179ji2ouUIGkiyLJ88efLs2bPDw8OhUGhubk6W5b6+PsPDent7T58+zW6nUqlTp04Fg8H9+\/e3fH0BSmllbwK0WKXuvo2NDU2wKa0EDSQiikajvIXETow1PwlJqJkaANgFkbZ12on13Ye2JknSxsaG02thJFAgjY6Ojo6Omi8BcAGNNB\/CB6BmYo1oAbibrjGEORoAaoVAskGbzI3YJpvZCN2FYRFFNmiTXa5NNtMKBBKAnXDJCYC6eS2Qksmk06sAwOFcJLAEp0UyeBcahT0JjPRldRoSCcAqr7WQABy1bY7vbWXfGFkCqMZTgYRTkcB5gp1p6AHohGgS0c6KJY8FEjQJjgj1wMUnAGqEQAKwn4bZGQBqh0ACsBvSCKAuCCQA++hqGjStXB4hogAqw9gAQJ34rP788o9aR4FUw9X5NN3kDdvkcrl0Ou2u66cBNJXXAgknxkKTlMaP\/urD7Ao0vQ9EfrKcKf6Ftm3ehpL6O1Yqor9+mhsv8QlgI68FkiMwFZX36OOHtl+A2HD1YX1sbJUjFs894iG0LZqKDw6Hw\/y1VldXqeQSn4goaCueCqSenp7bt287vRbgPlXjh12A2NLVh7XNq5iXHS0qu5DF2N69e\/XrU+kq1Igo78EvWs5TgQRgBfv+K4ry\/vvvd3Z2dnR0VIofpu4X2nbJ2FpOQix7FWrziCoUCu+\/\/34wGEREgRUCnhVLCKTGdXZ2Or0KYIa3foiIDf+w26qqrq+vv\/322zt37ty5c6csy5Ik7d69u5EDOp8kyEdbQ0i2MEQUy6fV1dWVlZX19fW1tbXV1VVJkn7yk5+srKzwB3d2dlpq1QGIAYEEXsMjRx8\/VOwZ8\/v9vPXzUz\/1U3zkhj1MURTSHf3rOKCzNNL4HHZ2ZBLP1I2NDRZFfKP8fr8kSQ8++CAb0GJbTbpWFN92RJTI8LuWQSCBu1mPn7Kdb1VHbvizsUN\/ncM2NcaSyUZVHdCq2tGnfypEFAgFgQRu0mD8mKs6cqMoCh+2oWLrxBhRbJZvn0\/TitPZVUsjXkRu+0ZZHIsyPBgRBU5BIIG4rBypqdh8acYKlD2g67vO9BElSZK6wculNvvsTMKInRhbulGyLJcWlDd1ixBRIAivBVIqlXJ6FaBOjsePFZIkybKsX8IjKpfLbdXvWpjOjm1XU+PHilojirULEVHQDF4LJHCLqvHj+JHaIn1Ebe+72+qsq3T6ETsxVjRWui71D0ZEgV08FUi9vb2vv\/6602sBZXgmfgxKxrSyvOqbq9RGyuVyb7\/9Nh+Iqm\/cqzUQUdAagn4BwNUqxQ8Vj1bujR\/zWJWkzoozNZTkEmta6WslqPFyvlZBREEzIJCgUW0bP6VjWpJ+LjtNV9WgT6PibUmSeNG5\/oUMtRJExLoEvRpR7IYDawziwX5gD1VVPf+lYttoHj\/sV7+X4ofqitVt3XbWChwqlfOxQ7mqqq6by66+iNL3XqptMMmbU9tYKBQceV1zHj+GQoP4D\/ZMJuPz+To6OtTt11xwb\/yQ6bUk7NgubVthA19mjXk5n0unW7USUfreSz5Bn8gDbGAjfMawxdBK4IdpVVXz+Xw6nebTvrE539xyjFArTGdHpteSqMf2rjl7Z680RJRq4aIV4o\/ZGCJKVdXV1VWWTGyCvnw+zyboowqtKPASr32iuECfdVVbCfrDtGHaN3ZAF3aEw8rwT3OO1Nq2y\/LZOr+qgWTtohX8wQLWSph8TLIsB4PBj3zkI2xtyw6wkYciyr1rbi+8C41yy55UU\/yU\/rl+BJ50xz4RfptbjJ+WHbCKRXa+6g+1lUmHmCC1Eqpungv9ecRVR+ksbheJGr1gkTsOphb19PQ4vQqiaDB+qirtaaGS7iP+SHsPfJXih0Qr6hPgcjMmtRLsBv+wmjEQZfJDocE9sNboRUS5hacCqW2pdV1v20Ym3Ufs6KCWFIlZbEJVTVZR4qdI1zJqZoddvaRm1kqwpyr7Sfn9\/mAw2Lwib\/PoRUS5BQLJfarGj2T9ettNU1MTSj8MQI4mq434ZN+b14oVLps2SfXWSkjFcwCaWydSL5PoRUQJC4EkOrVahZhk0\/W2m6pSE2p1dZUdF+7evZvP59fX1zs6OiRJ2rlzZ1dXVzOrD5qj2smw4jP5pIhoZWXl1q1b+XxeVdV8Pt\/R0dHV1dXV1bVnz55gMCh4ZQEiSnyC7jptyzx+yIkhehtVGlTw+\/3sdzc7xqmqurGxwR6fyWQkFw5T8447u2u\/W6fshyVJUnd3t8\/nIyJ2Xpqmaaw8YWVlhV093UUfVq0RJWBBqce47IjmPfyr3j7xo980qnw5CZNh6lrHNlpu8zrmTq9GDQxjdbXuh+Y1BS46jptHlKEMhFy1aQaaJuL+6bJjnNu1YfxQvcVvJsPUpWMbIv4qLzdRgzhMfi5ItZeK1PRhCf97YhuTiCqd1YncHFEicNlRz4pkMhmJRJxeCyKXXHGuPqoTxW+lw+\/iNqE0sWrsGmwA1aH0w2IrYBJR4s8rwSCimseDgdR6\/HDs1fgh07ad5FBJlaBNqMpB1Mp8srcB1Dj9oZmvIVUrvGyHiNp2oeG2J3ogxWKxqakpIopEIhcvXmTH9Ep6e3tbtV5bX3hFUd5\/\/\/1UKnXfffd1FQWDQdHOj6mJlbadgF2LlZpQtH3izpY0oTTSfPpyO42amEgWG0BC7YpStamPKs0FLtRWVGISUYqi\/OQnP1lfX19fX2eVpeyIkc1m3bJ1zSPQ0aRUPB6fnZ2Nx+OhUCgWi01MTExPTxs+5tYwP0B\/+MMfXltb8\/v9GxsbbKhQURRJklRVZbuX4DuZS+OnKoeaULoKu23\/sZNoDSBblP28KvXKknu6wgwflqqqbDPz+bzP55MkSdM0lr4C9Tk7RNzji6IoMzMzIyMjrFU0PDx85cqVpaWl\/v7+Zr90pW87WRufV4snbbCjnmEnc3wPa3DrXK11TSht6\/+17Uvq4MYGkC3Mf1IIe5ko\/Ury\/vyqH5aVYTYSYOuaStxAymaz6XT6wIED7G4gEJBleX5+3vZAsv0ALZXMQ6rfyUoHcpva5lPdNvVOKzW1CaVprLK2niaSajpi1+YfmYCXiVJ1cybVPbxaaZjN8a1rJXEDiYh2797d3d3NbsuyHA6HG3\/O1rcPSn+VG\/Ywdu4n2dEFoTo9qZ3bWW9CUaXPSyNNM5\/o25hObdsAskXpR0ZNvkyUyedl78+FNowooQOpqlQqZV7IYB4\/jvzYrHTIK929rOST+U9pSYBJ7Vyt1ibUxsZm63Mrcyq3jVRVzWQyaADZS6r3MlHszS\/7nFYSyN+S61XaGFE4MbZmq6ury8vLrI9OUZR0On3w4EH9Ay5fvnzu3Lmenp7e3t7BwcFUKsUeyb7nruieMhzyzPMpn893dnZ2dHR4rPrARcybUHx\/K3dWbJnvfy6XQwOo2cr+qjBp+Pr9fp\/PV+mL5vf72ax9gnxelSLKvI34zjvvdHV1PfTQQ86sdGXiHrYCgUA4HL5582Y0GiWibDarKMrQ0JD+MceOHWM3EonEuXPn2O1f\/uVf3rdv3yc+8Yl9+\/Y98sgjR44cEWTXsUL\/5eEd0ysrK2tra+l0en19nYjY9KP3339\/R0dHMBhkVxN3esXblOFg99M9P72Q+X91NXb8Vpk0kiRp\/\/79LVhJMCiNKPZFY9dNf++99+7evcuWs7M4HnjgAUmS9u7d64ovWtk2YjKZ\/NGPfpRMJl9\/\/fXbt2\/\/\/d\/\/\/enTp0+fPu3capbnW1tbc3odKorH42fPnmWnH8VisWvXrpmXfbMW0uXLl4kokUgkEgm2PBKJfOpTn2L\/NUSaOMyrD9g3gfV9s4lH2bRa+gdI7jlLw0sM3cJLmaX\/+N\/\/A2n0w3cWpR0dyUxSzea1vFb4oBCWe+6t3nto90P\/w8HDPaGeT31iyC2Fy95TtjOft5ACgQB7GOsHK+3qF3+oJplMJpPJVCo1Pz+fTCZfe+01trynp2ewqJUnblokdCBRjSfGGoicT1WL33i\/tvker+pKzMvWLwj7hXEvPoyklrsONxGZf3D6XtmyR7rmXcWunZX91PQJZP5dMwzVVPrgHOx6rZpAbFzDkXWzTvRAspGz+VS1+sCWwS3zfGp2ibknVaqLseuDM\/T4G8YF0YSqm1q5FNuWr5thIKq0XleSpKb+tiibQD09PUTkogQyaKNAMmhqPpkcxUj3i6zZ1Qf6kjAc6SzSH8hKO06l5k+wZthzDE1nx3+JC6vZCWRlBapGVCPfOE8mkEH7BpJBI\/lkJX5IgGlVTTqL2jafmt0AsmUNqVwTitr+g6PKpdgifHBUMvVRrT8K2yGBDBBI5VXNp4cffvijH\/2oWnJuk5VRBEG0YT5VrRxpdjeLLUwOcx5uQpn8dLByLpEg9J0WpSPHd+7cuX379rvvvts+CWSAQLJEn08vvfTSrVu32HJWXx6JRB555JFf+qVfcnANG6f\/trt6omU98RtAtig7dkje\/ew8M+FIMpl88803f\/zjH7\/22musGpstd1clgo0QSDVLJBKnTp36+te\/zmJJtPo9u5gMtgt7jKvaAGrN0J3jXNqEKptAkuVCOFeo2gt369atc+fOXbhwoa1yiPPy17Kpent7P\/e5z7Hbhv6973\/\/+2y5q\/NJKnd6Hc+n0omxHckn8waQZ45iteKbz5foe4oqzZ3jyMfHS09LS7G9MYGFPoFee+21ZDJJugT6+te\/bmgD8V+37QmBZAN2fhk\/7dmT+UTlZjE3ucpGM0rMTcaxCTMnmZLEuPq7ases2IIzT6Df+q3fardeuJrge2u\/ts2nXLlZzBs5wJk3gAScmdAtrDShbLl0oUkCeebjQwLZCGNINUulUpcvXz527Fh9E2+IPH+EjdTaT4Gq2gDyxvHLFfjvAKpwkjXV+Al6qYTEPIEarERo8PDidggkh7VPPpWWmKuq6vP5AoFALpfr6Ojg8+F77PjlDWV\/Yaiq2tnZ6fP5VFU1fIJuqaG3oqkJBHoIJLF4OJ\/0P59zudza2tr6+rqqqmwK866iYDAoZgkfkK4TdXV1NZfLsVmx2efIP0FZlsWfe9Qcq39LJpPz8\/OpVIonUG9vLyvIRgI1CQJJaK7OJ+vnALmxxLxNWD8ZqGwTilzyIeoTSD8tKRKoxRBIbiJyPtk7AlT2TE9nS8zbhHkptvUPsb5RqJZBAokJgeRi7TB\/OX8tzGLeJJUK4cjuQhJnm1BIIFdAIHmHCPOXt+YHb6USPhF+eovPpBS7lYUkJk0osmM2eiSQGyGQPKvBfDJpAIk2gWwdJebtw0Wl2CZNKCtdtUggD0AgtQsr+TQwMCBCA6hBZUvMydOzmOuZJJC++Sh+KbZJqQsRybL8zjvvLC4ull4gFQnkXgikbfgV02VZjsVi\/f39huV1XEldTJXyic1fvm\/fvkceeeTIkSMeOHDrexo9M4u5nvVCOLd78803X3311dKJsT2fQO1zXEIgbYnFYrOzs+xzjcfjZ8+eLb0di8WuXbs2PT3tsVH0l1566dlnn2Xnh4tWv2cvk6ELt+RT+yQQa\/pUagMR0eXLlz0\/MXZbHZdEb7a3jKIo165dGxkZYb8yBgcHw+HwwsLC4ODgzMwMXz48PHzlypWlpSX+I8Ub2Dwln\/vc59h326vz71HJNG6GfiFBZjE30CeQV2fFZkwS6NixY4Y2UCKRYLuoh7XbcQmBtEmW5QsXLvC72Ww2nU7zGwcOHGDLA4GALMvz8\/Nu\/+DNtcn8sFTtKhuls5i3Jp9MyjTapw1UmkBtqN2OSwik8ubm5sLh8ODgYDab3b17d3d3N1suy3I4HHZ23VqvffKJqs1iXnqVDVuyoR1mxWaQQI3w\/HEJgVRGLBY7f\/58LBaTZZkNNnheT0\/P6dOnWb98Ve2WT4YrCeVKrnRXa4m5eSl2uyUQC6E6nrmmndYD2uG41NaBxGtUiGh6ejoajZLuU+eN39XV1eXlZXZXUZR0On3w4EGn1rlJent7ecDU8bfUxvnEh3YMlxHS55N5KTZLIFeUYlvRvAQyaGSnFVk7H5e88AWo2+jo6OjoqH7J5OTk9evXX375ZV5AGQgEwuHwzZs32W6RzWYVRXHv8bQF2i2fDCUSPJ\/eeecdNpd5Pp9nk2FLkrRz585gMBgMBkWolbBLyxKoTbTzcamtA8kgFotdv37dUM4vy\/LJkyfPnj07PDwcCoXm5uZkWe7r63NwPd2lTfKptBS7q6uro6PD7\/f7fD6\/37+xscEuF6QoiiRJ7JEujSUkUCu11XEJ5yFtymQyJ06cYBc+4cbHx9lPFe+dgCYIkecvN2deil12EMhk6gFxSsxLsS+FyflASKDmabfjEgIJBCJyPjWjFFvMq2yYJBCvgkMhHDQDAgnEJcL1NVo5J6n5LOZ2lZiXQgKBIBBI4BrNvr4GCTYrdvNmMUcCgZgQSOBWjV9fg6olkFCl2I3MYo4EAldAILUvj42IWsmnn\/u5n6t0MpArplXVM8+nd955x+\/3v\/76655PIEVRxsbG2Mc9PDx85swZp9cI6odAalOenCpYr32ur8G8+eabqqq++uqrr7\/+uuHqDLwKzhsJZDA5OUlEZ86cYcl05MgRw0k84CIIpHZk+OpmMpnTp09\/7Wtfc\/vMjCZSqVQikWCX2Lh165ZQ9Xv1Me+Fo\/a4NMPi4uLv\/d7vffOb32S7bjwen5mZ8dhPq7YiSv94qb\/49x2\/MdKlX\/LVP9j43d\/bcGp9vMSrUwWb6O3tvXXrFhUvseHG83PNE+jrX\/96u12agYiWl5d9Ph+fY7S7u\/vWrVseuApD2xI3kP6XX86n1+6x2\/\/im51\/8sfSP\/o11dlV8hJPThVsnSvmj6gpgdpWOBwOBALsdnd39\/333+\/s+kAjxA0k7u\/\/844L35L+zZ+u79unOb0u4E2C5BMSCNqc6IF0+7Zv9PNdp76ofuJ\/LDi9Lp7iyamCzVm\/WkHL8imZTCaTyVQqZXsCtc+lGdLpdDabZYNGy8vLd+\/edXqNoH5CB5Ki0G\/\/RtenfyGPoSN7eXWqYHN1X63AxnxqXgKVrrMnL81g0N3drWna8vIyO2lheXm5p6fHA3OMti2hA+lP\/rjz7Td9f\/QnSCObeXWq4NaoI59akEDtqb+\/\/+GHH37xxRdZ2ffMzMyRI0dQYude4pZ9\/8W\/7\/jtX+\/68\/+QQ2ddk3jsxFhBVDr\/yfPnAzkFJ8Z6ibiBNPbkfd\/7zrYG3D\/6NXX6\/AdOrQ9AHVg+4dIMAFaIG0gAANBWdji9AgAAAETCBtInPxYI79r5yY8FnF4RAABoEUEDCQAA2g0CCVwvFosNDAwMDAwMDQ0tLi6WLj969Ggmk3FwDR2UyWSOHj3K3gc2MXbp8lgs5uAaAnAIJHC3WCw2Ozsbj8cXFhaee+65p556imVPPB7ny0dGRiYmJhRFcXplWy2TyZw4cWJkZGRhYWF+fj6dTrPsURRlYmKCLedvlNMrC4BAAjdTFOXatWsjIyPsJKrBwcFwOLywsMDOkeTLh4eHFUVZWlpyen1bbWFhgYiGh4epeDb0tWvXFEVJJBLpdJotD4VCIyMjr7zyisPrCiD4TA0A5mRZvnDhAr\/LLqtBbXl9jbKi0SibHYq5efMmv6GfJPvAgQOzs7OZTAYnR4Oz0EIC75ibmwuHw2wehDa\/vkapTCYzOzt78uRJNrNOOBzmU+x0d3fv3r3b0bUDIEILCTwjFoudP38+FovJspzNZp1eHbGwwaTDhw\/rG0wAokELCbyApxHvlGPX12C32fU1nFs7h\/E00s\/zlk6neZXH8vLy6uqqQ2sHsAWBBK43OTk5Ozv78ssv8zTi19dgd9vk+hplLS4uPv744yMjI\/o0OnDgALuMELt78+bNw4cPYwAJHIdAAjPsbJVTp04ZaqbZKT4i1ArHYrHr168bZitnFWVsoJ6I2vb6GplM5qmnnnryySdHR0f1y1k54tzcHBXHlh577DGH1nFLPB43nCzFF4qwp0ELCDq56ic\/Fnh7ybe\/T\/vbH2IwwGGLi4ujo6P6gxo\/u8VwmGs9tibsyt\/c+Pg4WzFcX4O\/Axx\/K23eP54AAATFSURBVPRvHX\/HHBePx8fGxqanp9lYlzh7GrQGAgmqM4zQTE5OptPp6elpXAkNbDc5OcmavIFAYGxsjIiwp7UPdNlBdcePHz906NALL7ygKEo8Hr969erTTz+NYwQ0w\/j4OBFNTU1dunQpnU4\/\/\/zz2NPaB8q+oTpZlp9++unR0dFXX331+9\/\/\/pNPPtluZ5hCy4RCoWeeeYa3jdqwo7WdoYUElvT39z\/55JNf\/vKXiej48eNOrw542cDAQCQSiUQiAwMDTq8LtBQCCawaHh6ORCL8VH+AJpmamtI0bWVlxVCRAZ6HLjsAEEgsFrt69WosFlteXh4bGzt48CBK7NoHWkgAIIrFxcXz58+zQcpoNDo8PHz+\/Hn9Na7A2xBIACAERVFeeOGFQ4cO8UHK8fHxYDDIyjudXTdoDQQSAAhhbGzMUOfNKu4SiYR+3iPwMJwYCwAAQkALCQAAhIBAAgAAISCQAABACKKch5RPJYlo47V5Itr427\/5p2vprxz8xz\/6he9+8f\/u+fD9kb49kb49kV\/Y\/2mnVxMAAJrFgaKGfCpZSCbzqWQh9Ta7vfG386UPmzqs\/fOHjQv79kSI6Bf2fxopBQDgMfa3kLLn\/+jet\/7lnj\/5rjTwcU1RVv7JiM\/v7\/jIASKqlD3WLd1JEtHSnVn9Qn1KEdEvfnjIkFLH\/\/w3iejSr\/xrIrr7wepn\/vTz0b4jv\/8\/fbmRNQEAAHs1pYW0OjFORLufn8qe\/yPlD5\/zaUQ+n8njd\/RGOnojOyKRzk9+mog6eiOdnxqiYvy8dfft\/\/TWPBH91dt\/89bdJFtoom9P5Idf+Gv9kks3\/vyr8TPxJ17av6f3B8nXfufKMy+PvLh\/T29jWwkAAHZqyhhS4OT\/euc3Pq988\/mNf\/dyR\/dDheVltnxHb4SIePZ09EaIiGVPWazpo+uXG2fLl+4ka0qpn+\/5JBH99a2\/Pb7nV\/7ijfhHH\/xppBEAgGiaEkjSwMd3fvF31v7wn+3637\/sezBExRBiCdS47aNHxpRiMaa3f0\/vw+FPLC7\/VzpE\/\/W9H\/\/qz37GltUAAAAbNbfKTv3v\/233k7\/d1JfgzGscfvVnP\/MvErHFzI9urd5mDSYAABBKU85DUhf+v9yffkf+p9\/44Oor6\/\/xL5vxErX6+Z5Prn6g\/Ov\/8u2e3fvQXwcAICD7W0iaoijPf8P\/j3\/N\/\/l\/oimr9577fWng4x0PPWT7C9Vk\/57ejz7403\/8X2b+zWf\/L2fXBAAAyrK\/haT8\/v+h3bp13\/CvEpH\/8yd8PT33\/s9\/Zvur1OFXf\/Yz4V0Pob8OAEBM9reQdj+\/ddVhnywHZ\/7U9peo28PhT6C\/DgBATG00l92\/\/dG\/Q30dAICw2iKQfpB87aF\/foiIjh\/6FafXBQAAyhP0An0AANBu2qKFBAAA4kMgAQCAEBBIAAAgBAQSAAAIAYEEAABCQCABAIAQEEgAACAEBBIAAAgBgQQAAEJAIAEAgBAQSAAAIAQEEgAACAGBBAAAQkAgAQCAEBBIAAAgBAQSAAAIAYEEAABCQCABAIAQEEgAACAEBBIAAAgBgQQAAEJAIAEAgBAQSAAAIAQEEgAACAGBBAAAQkAgAQCAEBBIAAAgBAQSAAAI4f8H7qjeVKlLv9sAAAAASUVORK5CYII=","height":420,"width":560}}
%---
%[output:4ff04843]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAI0AAAGECAIAAACERDSOAAAAB3RJTUUH6gQaESsZvtAfkQAAHANJREFUeJztnX9sG9d9wL8UTyElnWKlHmVRluDEztqq4mZMLhslUYGqqAo1ae02m6OkqQVoXNJBrYA5NZxGwQCvqJU18KyuDFwkHuvWXZoyWdbEWQa33sIiUVs5mtUaISEHSVSrpkxFdCwpOlliRN7tj6\/1cj7+Fn9IX+b7gWFQFHn37n3u+9733r2nMy0sLACz7ilb6wIwGcGeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaPCBJ4\/H44jD5\/Ml+6aiKD09PT09PYqi5FKCQCDQ2trq8Xjwx4MHDwYCAQDo7+\/v7OwMh8Npv4tFTfthsUH9HlPvJZMypCUcDnd2dooqFUeaFZJ45XK5XC5XOBzes2eP3W53u92yLKf4pizLx44dW8UuDTQ3Nw8PD+Nrj8fz4osvfvnLX87kiz6fr6+vb+\/evS6XCwD6+\/u\/9KUveTye5ubmzPc+MDCw6t9mQiAQcLlc999\/PxYSywwA+GPmpGr38ETo7+\/HHTgcDv3rkydPinjCd37wgx\/g2W04ZfC3GJr9\/f36152dnUNDQ3h2+3y+wcFBRVG6urpEHO\/fv9\/hcLS2tmKQCRRFOX78uNPp7OrqwncGBgaGh4eFJNyRw+FIHfH6iIn\/iv63+rAQB4gfOHjwYLJCHjp0qKmpSRSyvb3d7\/cLSaIN0xdSvCl27fP5Unmy2WwtLS2jo6PhcHh8fBwAQqGQoijj4+MNDQ1NTU2Gz09NTQ0PD+\/cufPZZ5\/VtxUOh6OhoeHUqVOKooRCIQAYHx\/H1y0tLTfccIM4hr1798qy7PV629vbASAYDHZ3d\/t8vpqamqeeekq\/r4mJibGxsba2toRB39\/fPzo66vP5fD5fKBTKJCx8Pt+JEyfcbrfX6x0bG\/N6vfrfYjPT0tLi9\/vdbvfg4KA4k4LB4G233baKQuJ56Xa79YX0eDxHjx71er3Dw8N2u33\/\/v3oL00e0dHRMTs7Oz09\/fbbbwPA2NjY2NjY0NBQS0vLxo0bDR\/etm0b\/o9fEe+j71AodOnSJfQ0NDSEm+ro6Eixd1mWa2trKyoq7HY7niKpS4uEw+HR0dGWlhabzWaz2Xbv3v3yyy8bzvRk9PX1TU9PDw8PG9olv98fDAaxtE6n0+l0Hj9+HMuTYSH14YjRc+rUqYaGBofDIepnenp6aGioqalpy5Ytsix3d3ePjIyMjIxAWk+1tbUA8Pzzz4+Ojt577701NTW\/+93vxsbGUImBrVu3JtvOtm3bxsbGTpw4MTs7+8ADD4RCoV\/96ldi+8moqalJ\/YGETE9Pz87OJixhCrD2AaCvry8+gRofH0cfWRWytra2pqYGT3GbzXby5Mnh4WHcy5UrV0KhUDAYbG9vdzgcJ06cCIVC7777bigUstvt8fGXxtOWLVuamppeeeUVTdM+\/\/nP2+32J598EgBaW1szrQAA8flXXnmlqanp1ltvnZ2dffrpp\/HEyWo7hoINDQ3Fn7\/62skcTIv8fv\/evXsB4Hvf+56+6d66dauiKPpGIhNSBFllZaXdbm9oaPD5fH6\/3+\/3nzx5srGxMdnn03iSZbmtrW1yctJkMt14441tbW0AsIr6xWo9d+6c3W5vamrCvi1Zw50JolkQHUl\/fz\/25PpuNRwOP\/vss5\/97GfTJoEejwe\/3tXV5XQ67XZ7RUWF+K3oYgEA26Lu7u60hZdled++fWNjY6KDHBgYwHYMADo6OoLBoN\/vF\/ka1vbY2NjExIRIlDD+pKQ7WaG1tVWWZWzusWXDwMzqsglLMDIy0tHRIcuy3W6HRO1ka2vr0aNHu7q68KROTXt7u9frdblcg4ODuAuRlA8MDPT392MysnPnzkzyCJfL9fbbb2NihpvSa7DZbD\/96U\/37NnjcDgAwO1248bT0tzc\/OKLL4ovAkBDQ8Njjz0myzLmTZimO51OzKWxX8RiOJ1OcXVkWlhYyGR\/zNrC40Y0YE80YE80SJ9HkCMajeL\/kiRJUokcILHDQAf4Al9HIhEAiMVi+H80GjWbzQBgMpkAQNM0VVUlSdI0zWw2S5JkNps1TRMKqbhcX0UUoSBeYNWbTCbxoqysLBqNLi4uVlVVlZeXYy1bLBZR7+JN3MLy8rJhm9FodGlpKRqNLiwsVFRUlJWV4RfRqNlsRtPrymXx8vJ4B1jvKAMDAjXgC5PJVF5eDtfWF74DAKFQqLGx0Wq1rrowS0tLwWCwrq5OkqSELjVN0zQNVoISADAWUaRBYaFd5mfTCR2ArjkSGlRVXVxcrKysrKioMJlMkiRhXQsH0gqpd5djpeAuLBaLLMvJNiWOBZLE5fLysiEo0SIAlJWVxSvMxWX6ryVzoI8G7AxExyBJEkYDDr3oHUSj0RxDoWiIyk3xGbw7Y7PZrFZrvEsUiXGJEYmv9f1lhi6lycnJaDS6adMm3PTS0hIkcmCIg\/LycuwG9A5S7EaQeyisK\/B4UwQlpIxLTdPwneXlZUVRqqqqysrKsLHV5zuSJEkvvPDCq6++um\/fPrvdLuJAdMUiIBYXFy9dukQiDn5zZuiHv378umdabypvvvzwC\/f8+ddu+Wh2o\/v5JZe4xBfLy8tSfX3966+\/bjKZbr755kz2t\/45O\/f7j31xm\/nWmZ\/87IG5f5mvu6dhbT1lAsaM1WpNFgbS1WFzIg70iGupeGJazKSV1FBLSR2MQFW1mKam\/kwKzeuQtQmjHOsobfSrWsykmbR8bGqdQKOU2aKqqklNE095oWhBWWxPxTl\/VU0FLQaZBFQ+KMJBlWY8xdQYaKa1LkU+ueppcnJybcuRX1RVAy221qXIJ9LmzZvXugz5R9ViWrEaveJAuN3Da\/WEqKpq0gBSdlApvr4OIewpBaqqgppaEwCdpBzoekpdxTE1ZipKw1e0oFwbT4U9PE0Tnkqml1oDT0VobVRVBbVIiorTeFJt91ITU2NQWuOwVz0Fg8G1LUd+UTUVNK1o4xFFoETjKaZCMYb3iocEAPX19WtdjNWQbAxUA1BjMS2WZryc72usPTE1BrFSHN8rMrmfy6myLA3UmJrJ8F7uqVrJ3tcoDjH0xHlETrssxvVTrGjD5Xz9tHpiUVWLaaUUUOSvn6IrLC4u4o+zMzOqFlNX7mwoihIOh8V0xPU\/\/zAhlOJJr+Ty5csAoF\/LjSasVqskSeqSGoup4v1oNIqfnJ2dFW\/OzMxYLBar1VpRUbEeVmSkZp0WLj5KcEI1glUvSVJNTQ1O3dVHiVxdHbsSi8WujkdYrVZcXi8mZuMo8OXLl+fm5q677jq9PLEpnAi8foJPAoC1vaWrV6Kf446I+dU2mw1\/xLrD+b3ibyNdgwZq7Oq8ez0YMeKvDeBe7Ha7\/oRYWlpSFCUajRrkiZn0hnOiaBQvnsRE+CtXrkR1fFCUJEpWAS6QSPsx\/doFw74M8kTLqf+i1Wq9fPmy2WxeWloqtLyCeNKHBUaJXsm7774rSdL111+PfQPkpiQhmqZquY3DppAnVj5Fo9G5uTlYWVCk7yAh3zlLfi7IkykRJ6zorgFAluVoNIp9RoHQVE0rwP2nZOmGzWbDbg+PPT5n0Xd7q8tZsviCvhCZKElWoPn5+WxLmSWapmrq1bS84JdQYgmU\/k1DzrK0tITaksnTd8mJ95Lw3fgomZubC4fDsVhMrFjGAE+tZE3QAFRN01RtDe+6G3IWQbKcJRKJYPVWVVUlzFmkI0eOTE5Oqqo6MzODWzGkWwBgtVqxsaqrq0u9dq6YJB0DRVHY7q2zEYlk3d78\/HwkEpFlGVcYxucsktPpPHLkiM1mw9+hkvgowRAWneQ6B1fEpv5MdN2sQMVm6frrr6+pqREKDTmL5HQ6d+3aNTU11djYuLbFzYrUVazinxSgPH\/P0JWszWSPQt+20VRNVYvROxXt\/lNJTcq5igbiT3SUDGvgqQitTYGunxJSnMazFONJ5BElFFEl6klV1WLFU3G46oniOrUUfbimQdr+qWgpQF4oxXjSrg4ccbuXK7mfy+IPvCUkwzwi9UbWFWWw1vcJ8465okzTAExXL3WlCnPh9lXK108FP4vx+qkk84hSAhfmFu0ytziNZwl6AlhJ9kookyhNTwBplnzSSspBeCI6zzJxdWvXtHulEVKlG08ZsJ7vaxig6il1FWsaXF33WchoKmbjWQZrtJ6wsAdZarc1yMZTKoxJeWGNlex9jcIfmLaSkpdOTJHpSFMgZrHhFLaZmVkckkBNyrwyMzMDlBfVAEVPaAVXNVVUVGiaJiayiTlS2rVzwvRzHPEDZWVlMzMz+vk96xwCnsSUQgwX8cSCWCz2xz\/+sbKysrKyUpZlnIeL89pANxdWluXGxkacI4czGhcWFubn5yVJunTp0uzsrJhTt4bLMdLygadgMNjQ0LCGRUEMy2z0sQIAVqvVZrNhzd500004BXpphatxMzOrHzJSFOXChQv4Gucfbty4UUx2F1NTxYxi0M2GXD\/m1vjvEmRiBZIv6MD1T\/hFfETFwsJCJBLRp3z4TlVVlX7+qNiCfrOGScUGc+hYb86wLqigFNWTqIj5+fnXXnsNACRJslgsFoulqqpKPxU3xSmcWu3GjRs3bNhw9ToXAAA2bNiwadMmrHf9NHwxP\/6DOdxxk4rjzeHj2yKRCD4nxmKx4GcKncQWcD2hIQ3Tn30f+chHIpEIHtvy8jIeLda4tPLULP05m1XAWa1WEImEBlarVSw71G8Kp3GL5AJWpu0Lc1Hdc5Rw1\/gx8Qwxs9msqqp4vHhC9\/kib2eB3gpcu6JWnKfJYkXfcL3zzjvRaDQSieCD7nB5SFYBhyQckEgYMbhrDBd8upw+XACgsrKypqYm2a5Tu8+XuezWPxl+jE\/D4Nrlm2lbMIgLOIvFYjabrVZrLBaLDziMNn3AJSSTQSPD3rH8VVVVGCv6XQOAoiiSJEWjUavVuri4mLa1NCxbSxG14iupu7osPKmq+sYbb+Dj0vAA9GuhRBqWrKXOS8AZFnwlbmqSDMKmaEIlSUoRLrCyti5+7wnTQinRsjVRflzzdOnSJXwGWiQSsVgsiqKkXrAkJZy5l\/CQ8Jl7+AGz2YwHIGoq4RHmGHAflPLaIzd07\/oTVpKkpaVF0I3E4iOwkuX3afeOn9cvrI9PLvQfNizfM7xfXl6OtYG2sHUFAIvFUlZWduHCBX1NXhO1IyMjR44cgZUr\/GjcOrVkvUI0GhVnmaIo4gQxmUw4DR9bjMwDLnMSNjXz8\/Mrzub07d6CsjA3N1dVVXXDDTdkeFqsYu\/x50105amB+goRtYFPr9OvdoKVRWYJo\/aD5wHccccddXV127dvr6uru\/3229va2tImx3h2iDcxhYtEIlgm7GzwGWF5T4ES9i5YBbt2fKV5myM8Mx1bUv+s3PaJGz+BhcG\/npPwbM0Rcf5hnUiSpChKLBYT+YjhrMWOQ3++6rsucYDBYPCtt946f\/78mTNnrj4\/d3Jy8oUXXgCAkZGRkZER\/GhDQ8Mtt9yC\/3\/yk5+MrxTQJcSGgNNHW8J+KNs6St27iKYm2Wb1PUR8+Y09XMZFEvkC\/o0J0LUf8RUiCqDPGvSnztzc3NTU1OTk5PDwcDAYPH36NKzcHUzwnGPsseK1iWjbsWPHZz7zmazakITVlFobfiXhwvq0S+ozKY8+mY4vUsIRoxRHkfZESVgGDJfz58+fPXt2amrq7NmzsCLG6XQ6nc7NmzfjgwnTP496cnLy4sWLaCtZtLW2ZvegxvgDxk4VnwELAPjY4FxqIVsSNgDRaBQfJm5ou1ZdpGAwGAwGMWJOnz6N04eEGLSCYgxk\/dzwZNGWrTZ9o4HjctigR6+9tpVluby8vGjrt0UQLy0t4cAgqsLHEuOlCI7NZ1gqbL5QT7wYfcSkJtfnu2euTd+OpW40kjUvkOQiMRcyLJXhKiphqaqrqyVJEj4wYvBj9fX1mzdvrq+vz1yMgVw9GUihbceOHTU1Ndu3b7\/99tshLu9IgehLcteWLEvUi8kwdkWp3nrrrTNnzpw9e\/bSpUtnzpzB3+YuxkCePRnQa3v++ecvXryI7+fYt8VrA921sF5bCjH67DyrRhUjRp+SIWglX2IMmBYWFsLh8J49e7Dp3Lt3r8vlAgCPxzM4OAgAbre7vb099z1heO3atSv3vk2PQRuKiUQiqqpaLJalpSUxvrXq\/DuhGEPEjIyMJEsBVo3ei2lhYaG\/v3\/btm0ulysQCLhcrkcffbS2tvbAgQNHjhzx+\/3Hjx93u93xf6ond\/KSkhgiRn+BiYOQmJIkjLaEYL2kFoNucj3+dOi9XNPuKYrS19fX3d09Pj4+NDTkdrsXFxd7e3sPHDjQ3Nxc6GLlnpIkvLoU10mGPxqE4VVdXT01NQVJxGB8FE1MMhRFuaZdnpiYUBTF4XCMj4\/b7XZZlvHe6\/T0dBE8YUX09vbijwZtzz33HL5vSElSD6fqr\/ZtNhsG37lz5wDgpZdeiu\/8nU5nb29v3luwHJmYmPjAUzgcPnDgwIEDB\/AO6ZqTTJsY4kIyaSSTNWXIOhSjB71IPp+vvb09EAhghyQkhUIhbCiqq6tra2tXt488JiNC2+TkJCZU9fX1CaNNaDMMlMFKxHz3u9\/FLaDyXbt25atN8\/l8fX19oEvHsqW\/v\/\/EiROwUmPCi2lhYSEQCDzxxBMDAwMiWRC\/ziWPyMtGMiRZ31bQXNlAOBzGvhwADCf96tB7kRRFOXTo0MjIiGg00GRnZ2d7e7ssyx6PZ3X1Ozw8jDeuHA6HoigTExOF6+QSNpJF7vn9fv\/8\/HxtbW1FRYUsy36\/P5cmxOBFkmX52LFj8Z9zuVyri1w9xU9GkLXKzex2O\/4FaQAYHx\/PxZPBSymuqylFCutpFclIOBzu7Ox0OBwOh8Pj8eCbHo8H3\/H5fHkvpKIoPT09uP3+\/n5DMXp6evQXXqkJhULiBvfWrVvzWMgCemptbVUUZXFx0e\/3y7K8ZcuWTL41ODi4e\/duv9\/v9XqPHj3q8\/kCgcDJkyd9Pp\/b7T5+\/HjmtZYhXq\/Xbrf7\/X6fzzc6OoonBxZjeHgYAERWkhqHw1FdXT09PS0uQ\/NYyALe1Glubl5FMjIwMIAvtmzZ0tTUBIXPR0Q3bLPZWlpaACAcDo+OjnZ0dMiy3NbWdurUqUx6GpvN1tvb29XVBQButzu\/l6GFvfmWSzJS\/MGRcDj85ptv3nfffXBtQ42tdybnWXt7u9\/vL0TZ1mkeUfzBEUVR9u\/f39vbW7SkNCvWo6dAINDb23vkyBFRZXkZHElBOBzu6enZt2+faN8wcPE1RnPed5oV684TXoQfO3ZMRNLq8pHMCYfD+\/fv158W2FGNj48rijI0NNTR0ZHfPa6Cwt7PzRa8saLPr3BwBMcJMR\/Je7skhtQQHJoT9+h27twpUps1JAtPWPTdu3eL1CD+HSYtPp\/v4YcfFiecuDebOqXMLp4CgcCDDz54+PBh3AdeFa6H040WHo8Hb8MCQF9fX1tbW9oTPbv+qbm5+e677z506JCiKIFA4M0339y7d+\/qy\/thBa+xvF6v1+sVP6Ym6\/4J27pvfvObzz33XHd3d16muHwIweYOADLscbPO92w220MPPfTtb38bANbtPdD1D462NDU1ZZi+riYvxztv3d3da35VQRev16soiqIo2PSlhcDfYyk9AoHAM888c\/jwYQB48MEHW1tb0zZ96+46t+QJh8Pf+ta37r777ubmZn1elvpb7KnY7N+\/3263ixwPX6S9tllf4xFMMjieaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaMCeaJCpp1\/+t9leVan\/9\/jh8oKWjNFjWlhYyPY7jx8u\/\/GT0n\/9eqmuTitEmZh4pGy\/cPb3ZceekH708whLKibZ9U9TUybXvZaer0e3\/5VaoAIxCcmi3VMU2PM31oZG1X30\/YKWiYknfbv321fL\/uHrFgDYvkO9cN70wx8vF75UjJH0ni5MlF2YMGkA74TMJ17m3GFtyKJ\/ev996Gyziry87\/7rClcsxkCm+Z4J4PtPvN\/1tWhBS8Mkg8cjaMCeaMCeaGB+5JFH9D8vHv3he1\/vkW7+qDo9vfzKr+f+\/m\/tp38iLc69esfTv686OgsXXr0wbDIBgKnGej0AdP3i7\/7j3IndTTsB4L335z\/31F+fn\/1T+41ta3EspUyC69zZe74SPfN\/YCoD0DQAkwZgMm2535iOn7zH++nGW71jv3jEN+C77\/nGDZt\/Ezz9jZMPvbj7qcYNm4tV\/g8LCdq9yge+ASbQNA3AZAITmExlmxuSff+2+k8BwG8vvgYAv3zb97GNN7OkQpAgL7\/us5+THH8Z9fstX7jT+tXusoYG8+aGBYCJueCf3rswMRecmAv+6b3gpxtvBYDGDZt32LcHpt+AJnjj8lt3ffzOoh\/Ch4LE10+WL3wx6veDxVJ+S6t4c8uGhi0bGj7daPzwXR+\/8\/ERTyB87uL8FIYXk3cStHtR\/+tLP\/+Z\/E8H33\/5VOR\/\/yftJm6r\/9T8+8q\/\/eHf66vruNErEMZ40hRFeeyg9Z6vWu\/9mqbMX3n0O5LjL8ybNqXYROOGzR\/bePOTfzj+oy\/+ayGL+qHGGE\/Kd\/5Ru3jxup13AYD13j2m+vor\/\/LPabdy18fvtFdt4kavcBjjqfqxQfHaJMs1x3+e4YZ22Ldzo1c48jMe8Z\/nXuJMr6Dk6uk3wdObvt8EAF1NX8lHeZjErGa+EVN8eByWBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBv8PaobB8aLWkRIAAAAASUVORK5CYII=","height":388,"width":141}}
%---
%[output:894ff500]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAI0AAAGECAIAAACERDSOAAAAB3RJTUUH6gQaESsZvtAfkQAAG\/NJREFUeJztnX9sG9d9wL8kTyElnWOlHmVTP+DUztqq4mZMLhslUYEqqAo1ae02m6OmqQVoWtJBrYA5NZxGwQCvqJU18Kyu7FwkHuvObZYyWdbEWQa33sIiUVs5mtUaISEHSVSrpkxFdGwpOlpiRN7tj6\/1fD7+Fn9IX+b7gWGQFO\/u3fvc973v3b3HM0UiEWDWPObVLgCTFeyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBtc8eTweZwI+ny\/Vkoqi9PT09PT0KIqSTwkCgUBra6vH48G3Bw4cCAQCADAwMNDZ2RkOhzMui0XN+GWxQv0W028lmzJkJBwOd3Z2iioVe5oTknjV29vb29sbDod3797tcDjcbrcsy2mWlGX56NGjK9ikgebm5pGREXzt8XhefPHFL37xi9ks6PP5+vv79+zZ09vbCwADAwNf+MIXPB5Pc3Nz9lsfHBxc8V+zIRAI9Pb2PvDAA1hILDMA4NvsSdfu4YEwMDCAG3A6nfrXJ06cEPGEn3z\/+9\/Ho9twyOBfMTQHBgb0rzs7O4eHh\/Ho9vl8Q0NDiqJ0dXWJON63b5\/T6WxtbcUgEyiKcuzYMZfL1dXVhZ8MDg6OjIwISbghp9OZPuL1EZO4iP6v+rAQO4hfOHDgQKpCHjx4sKmpSRSyvb3d7\/cLSaIN0xdSfCg27fP50nmy2+0tLS1jY2PhcHhiYgIAQqGQoigTExMNDQ1NTU2G709PT4+MjOzYsePZZ5\/VtxVOp7OhoeHkyZOKooRCIQCYmJjA1y0tLTfddJPYhz179siy7PV629vbASAYDHZ3d\/t8vpqamqeeekq\/rcnJyfHx8ba2tqRBPzAwMDY25vP5fD5fKBTKJix8Pt\/x48fdbrfX6x0fH\/d6vfq\/YjPT0tLi9\/vdbvfQ0JA4koLB4O23376CQuJx6Xa79YX0eDxHjhzxer0jIyMOh2Pfvn3oL0Me0dHRMTs7OzMz8\/bbbwPA+Pj4+Pj48PBwS0vLhg0bDF\/eunUr\/o+LiM\/RdygUunjxInoaHh7GVXV0dKTZuizLtbW1lZWVDocDD5H0pUXC4fDY2FhLS4vdbrfb7bt27Xr55ZcNR3oq+vv7Z2ZmRkZGDO2S3+8PBoNYWpfL5XK5jh07huXJspD6cMToOXnyZENDg9PpFPUzMzMzPDzc1NS0efNmWZa7u7tHR0dHR0cho6fa2loAeP7558fGxu67776amprf\/va34+PjqMTAli1bUq1n69at4+Pjx48fn52dffDBB0Oh0C9\/+Uux\/lTU1NSk\/0JSZmZmZmdnk5YwDVj7ANDf35+YQE1MTKCPnApZW1tbU1ODh7jdbj9x4sTIyAhu5cqVK6FQKBgMtre3O53O48ePh0Khd999NxQKORyOxPjL4Gnz5s1NTU2vvPKKpmmf\/exnHQ7Hk08+CQCtra3ZVgCA+P4rr7zS1NR02223zc7OPv3003jg5LQeQ8GGh4cTj1997WQPpkV+v3\/Pnj0A8N3vflffdG\/ZskVRFH0jkQ1pgqyqqsrhcDQ0NPh8Pr\/f7\/f7T5w40djYmOr7GTzJstzW1jY1NWUymW6++ea2tjYAWEH9YrWePXvW4XA0NTVh35aq4c4G0SyIjmRgYAB7cn23Gg6Hn3322TvvvDNjEujxeHDxrq4ul8vlcDgqKyvFX0UXCwDYFnV3d2csvCzLe\/fuHR8fFx3k4OAgtmMA0NHREQwG\/X6\/yNewtsfHxycnJ0WihPEnpdzIMq2trbIsY3OPLRsGZk6nTViC0dHRjo4OWZYdDgckaydbW1uPHDnS1dWFB3V62tvbvV5vb2\/v0NAQbkIk5YODgwMDA5iM7NixI5s8ore39+2338bEDFel12C323\/yk5\/s3r3b6XQCgNvtxpVnpLm5+cUXXxQLAkBDQ8Pjjz8uyzLmTZimu1wuzKWxX8RiuFwucXZkikQi2WyPWV34uhEN2BMN2BMNMucR5IjFYvi\/JEmSVCY7SGw30AG+wNfRaBQA4vE4\/h+LxSwWCwCYTCYA0DRNVVVJkjRNs1gskiRZLBZN04RCKi7XVhFFKIgXWPUmk0m8MJvNsVhsYWGhurq6oqICa9lqtYp6Fx\/iGpaWlgzrjMVii4uLsVgsEolUVlaazWZcEI1aLBY0vaZcli4vT3SA9Y4yMCBQA74wmUwVFRVwfX3hJwAQCoUaGxttNtuKC7O4uBgMBjdt2iRJUlKXmqZpmgbLQQkAGIso0qCw2C4Ls+qkDkDXHAkNqqouLCxUVVVVVlaaTCZJkrCuhQNpmfSby7NScBNWq1WW5VSrEvsCKeJyaWnJEJRoEQDMZnOiwnxcZl4slQN9NGBnIDoGSZIwGvDSi95BLBbLMxRKhqjcNN\/BuzN2u91msyW6RJEYlxiR+FrfX2bpUpqamorFYhs3bsRVLy4uQjIHhjioqKjAbkDvIM1mBPmHwpoC9zdNUELauNQ0DT9ZWlpSFKW6utpsNmNjq893JEmSXnjhhVdffXXv3r0Oh0PEgeiKRUAsLCxcvHiRRBz8+vTwD3\/1Lzc8c+vWCue7j7zQ9af33\/qR3K7uF5Z84hJfLC0tSXV1da+\/\/rrJZLrllluy2d7a58zc75p3ftR8h\/Kjnz4wd3B+4331q+spGzBmbDZbqjAwX71sTsSBHnEu9UGgPK8baXA1pU4DLc2r4ynPOsoY\/RpoABk8ZbmqNUJ5xhMGVAm2U7KgLLWn0hy\/mqZmGU8FoQQ7VZ7xJC75lA1XPU1NTa1uOQqNppUwnkqAub6+frXLUHjUsgsoGtlOUvBcPSmqppo0dcWLr0EIe0pDXI2DZsrY8lFJyoGup\/RVHI\/HIUM4FYaSBeXqeCr27qmqir1T2XRRq+CpBK1NXI2btGwvSeRJaRpPqu1eemJqzKxayieahKdgMLi65Sgs8eV2r2wox3jSIK7G8T532SABQF1d3WoXYyWkvgaqqfG4Sc1wSYzWfY1yjKer509qGfZPJSb\/YznD+ROe5+a3kmwoWVCWbzypmT0RokzPn+JxiJtK0+zx+dMK0QDi8bgWX+1yFBTy50+xZRYWFvDt7OXZeFVcW+44FEUJh8NiOOLaH3+YFErxpFdy6dIlANDP5UYTNptNkqR4LK7Frl42wsHS+M3Z2Vnx5cuXL1utVpvNVllZuRZmZKRnjRYuMUpwQDWCVS9JUk1NDQ7d1UeJLMvxeVWNX+2ebDYbTq8XA7PxKvClS5fm5uZuuOEGvTyxKhwIvHaCTwKA1b2lq1eiH+OOiPHVdrsd32Ld4fhe8dtIBlQ1rsaN12ExYsSvDeBWHA6H\/oBYXFxUFCUWixnkiZH0hmOiZJQunsRA+CtXrsR0XCtKCiUrQFW1bK7w6ecuGLZlkCdaTv2CNpvt0qVLFotlcXGx2PKK4kkfFhgleiXvvvuuJEk33ngj9g2Qn5KkaKqqqnmNZEkjT8x8isVic3NzsDyhSN9BQqFzlsKckKdSIg5Y0V0DgCzLsVgM+4wioamaVoTrsKnSDbvdjt0e7ntizqLv9laWs+SwgL4Q2ShJVaD5+flcS5kbGqgYT1CKO4ViCpT+Q0POsri4iNpSydN3ycm3kvTTxCiZm5sLh8PxeFzMWMYAT69ktVBVTVXxOuzqXIs15CyCVDlLNBrF6q2urk6as0iHDx+emppSVfXy5cu4FkO6BQA2mw0bq02bNqWfO1dK0lwD1VRVyzQubFVI1e3Nz89Ho1FZlnGGYWLOIrlcrsOHD9vtdvwbKkmMEgxh0UmucVT12vlTKmJrZgYqNks33nhjTU2NUGjIWSSXy7Vz587p6enGxsbVLW5OpK1iTbva7mVQtUY8JcXQlZCc\/5QeTQO1OPleIiW7\/1Sm8zXyPn9aa6yCpxK0Nuq1xLzolKbxLM94UlVtbeZ7K6YcPWn4i0Rl1OqRnqeW\/vxJjWcYb1SyFKAglGM8LfdPq12KQkI1Lxc\/8JaULPPy9CtZU5hhte8TFhyLzbz8w3kAGkiVluJtq5zPn0pxFKuglWUeUW4s33MvgavSNJ7l6Em7+rs55TTjvQw9Xf1J1zJKykF4IjrOMlV1X53zWaKZn6WgDOMJINvfoVrL9zUMUPWUroqvl1S8iCpl42mGVZpPWOyd1MrrF46oxlN6SimpbO9rFH3HrmYQZRRNa3YeQE6IUWw4hO3y7GXQMOXTAECZn798+TJQnlQDFD2hFZzVVFlZqWmaGMh2deSbJBl+NEc\/xhG\/YzabL1++rB\/fs8Yh4EkMKcRwEU8siMfjf\/jDH6qqqqqqqmRZxnG4NputulrWn+jKstzY2Ihj5HBEYyQSmZ+flyTp4sWLs7OzYkzdKk7HyMg1T8FgsKGhYRWLghim2ehjBQBsNpvdbsea\/fCHP4xDoBeXEXGj6U5yFUU5f\/48rgTHH27YsEEMdhdDU8WIYtCNhlw75lb5dwmysQKpJ3Tg\/CdcEB9REYlEoovRa0mEBpFIJBqNVldX68ePijXoV2sYVGwwh4715gzzgopKST2Jipifn3\/ttdcAQJIkq9VqtVqrq6v1Q3HTHMLp1W7YsGH9+vXLeQQAwPr16zdu3Ij1rh+GL8bHXxvDnTCoONEcPr4tGo3ic2KsVit+p9hJbBHnExrSMP3R96EPfSgajeK+LS0t4d5ijUvLT83SH7M5BZzNZtN0ebnNZhPTDvWrwmHcIrmA5WH7wlxM9xwl3DR+TTxDzGKxqKoqHi+e1H2hKNhRoLcC18+oFcdpqljRN1zvvPNOLBaLRqP4oDucHpJTwAGkPHlKGjG4aQwXfLqcPlwAoKqqqqamJtWm07svlLnc5j8Z3iamYXD99M2MLRgkBJzVarVYLDabLR6PJwYcRps+4JKiZXEd1rB1LH91dTXGin7TAKAoiiRJsVjMZrMtLCxkbC0N09bSRK1YJH1Xl4MnVVXfeOMNfFwa7oB+LpRIw1K11AUJOMOEr+RNzXKyZzCVpgmVJClNuMDy3LrErSdNC6Vk09ZE+XHO08WLF\/EZaNFo1Gq1KoqSfsKSlHTkXtJdwmfu4RcsFgvugKippHuYZ8BdK+X1e27o3vUHrCRJi4sLcC2cNHwEVqr8PuPW8fv6ifWJyYX+y4bpe4bPKyoqsDbQFrauAGC1Ws1m8\/nz5\/U1eV3Ujo6OHj58GJbP8GMJ89RS9QqxWEwcZYqiiAPEZDLhr7xji5F9wGVP0qZmfn5+2dnc1dFGmgYAESUyNzdXXV190003ZXlYrGDricdNbPmpgfoKEbWBT6\/Tz3aC5UlmSaNWEsneXXfdtWnTpm3btm3atOmOO+5oa2vLmBzj0SE+xBQuGo1imbCzwWeEFTwFStq7YBXs3P6l5i3O8OxMfFH9kwr7x2\/+OBYGfz0n6dGaJ+L4wzqRJElRlHg8LvIRw1GLHYf+eNV3XWIHg8HgW2+9de7cudOnT199fu7U1NQLL7wAAKOjo6Ojo\/jVhoaGW2+9Ff\/\/xCc+kVgpoEuIDQGnj7ak\/VCudZS+dxFNTarV6nuIxPIbe7isiyTyBfyNCdC1H4kVIgqgzxr0h87c3Nz09PTU1NTIyEgwGDx16hQs3x1M8pxj7LEStYlo2759+6c\/\/emc2pCk1ZReGy6SdGJ9xin12ZRHn0wnFinpFaM0e5HxQElaBgyXc+fOnTlzZnp6+syZM7AsxuVyuVyu+vp6fDBh5udRT01NXbhwAW2lirbW1twe1Ji4w9ip4jNgAQAfG5xPLeRK0gYgFovhw8QNbdeKixQMBoPBIEbMqVOncPiQEINWUIyBnJ8bniractWmbzTwuhw26LHrz21lWa6oqCjZ\/G0RxIuLi3hhEFXhY4nxVASvzWdZKmy+UE+iGH3EpCff57tnr03fjqVvNFI1L5DiJDEfsiyV4SwqaanWrVsnSZLwgRGDX6urq6uvr6+rq8tejIF8PRlIo2379u01NTXbtm274447ICHvSIPoS\/LXlipL1IvJMnZFqd56663Tp0+fOXPm4sWLp0+fxr\/mL8ZAgT0Z0Gt7\/vnnL1y4gJ\/n2bclagPdubBeWxox+uw8p0YVI0afkiFopVBiDJgikUg4HN69ezc2nXv27Ont7QUAj8czNDQEAG63u729Pf8tYXjt3Lkz\/75Nj0EbiolGo6qqWq3WxcVFcX1rxfl3UjGGiBkdHU2VAqwYvRdTJBIZGBjYunVrb29vIBDo7e197LHHamtr9+\/ff\/jwYb\/ff+zYMbfbnfhTPflTkJTEEDH6E0y8CIkpSdJoSwrWS3ox6Cbf\/c+E3st17Z6iKP39\/d3d3RMTE8PDw263e2Fhoa+vb\/\/+\/c3NzcUuVv4pSdKzS3GeZPjRIAyvdevWTU9PQwoxGB8lE5MKRVGua5cnJycVRXE6nRMTEw6HQ5ZlvPc6MzNTAk9YEX19ffjWoO25557Dzw0pSfrLqfqzfbvdjsF39uxZAHjppZcSO3+Xy9XX11fwFixPJicnr3kKh8P79+\/fv38\/3iFddVJpE5e4kGwayVRNGbIGxehBL5LP52tvbw8EAtghCUmhUAgbinXr1tXW1q5sGwVMRoS2qakpTKjq6uqSRpvQZrhQBssR853vfAfXgMp37txZqDbN5\/P19\/eDLh3LlYGBgePHj8NyjQkvpkgkEggEnnjiicHBQZEsiD\/nk0cUZCVZkqpvK2qubCAcDmNfDgCGg35l6L1IiqIcPHhwdHRUNBposrOzs729XZZlj8ezsvodGRnBG1dOp1NRlMnJyeJ1ckkbyRL3\/H6\/f35+vra2trKyUpZlv9+fTxNi8CLJsnz06NHE7\/X29q4scvWUPhlBVis3czgc+AvSADAxMZGPJ4OX8pxXU34U19MKkpFwONzZ2el0Op1Op8fjwQ89Hg9+4vP5Cl5IRVF6enpw\/QMDA4Zi9PT06E+80hMKhcQN7i1bthSwkEX01NraqijKwsKC3++XZXnz5s3ZLDU0NLRr1y6\/3+\/1eo8cOeLz+QKBwIkTJ3w+n9vtPnbsWPa1liVer9fhcPj9fp\/PNzY2hgcHFmNkZAQARFaSHqfTuW7dupmZGXEaWsBCFvGmTnNz8wqSkcHBQXyxefPmpqYmKH4+Irphu93e0tICAOFweGxsrKOjQ5bltra2kydPZtPT2O32vr6+rq4uAHC73YU9DS3uzbd8kpHSXxwJh8Nvvvnm\/fffD9c31Nh6Z3Octbe3+\/3+YpRtjeYRpb84oijKvn37+vr6SpaU5sRa9BQIBPr6+g4fPiyqrCAXR9IQDod7enr27t0r2jcMXHyN0VzwjebEmvOEJ+FHjx4VkbSyfCR7wuHwvn379IcFdlQTExOKogwPD3d0dBR2iyuguPdzcwVvrOjzK7w4gtcJMR8peLskLqkheGlO3KPbsWOHSG1WkRw8YdF37dolUoPET5iM+Hy+Rx55RBxw4t5s+pQyt3gKBAIPPfTQoUOHcBt4VrgWDjdaeDwevA0LAP39\/W1tbRkP9Nz6p+bm5nvvvffgwYOKogQCgTfffHPPnj0rL+8HFTzH8nq9Xq9XvE1Pzv0TtnXf+MY3nnvuue7u7oIMcfkAgs0dAGTZ4+ac79nt9ocffvhb3\/oWAKzZe6BrH7za0tTUlGX6upK8HO+8dXd3r\/pZBV28Xq+iKIqiYNOXEQK\/x1J+BAKBZ5555tChQwDw0EMPtba2Zmz61tx5btkTDoe\/+c1v3nvvvc3Nzfq8LP1S7KnU7Nu3z+FwiBwPX2Q8t1lb1yOYVHA80YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90SBbT7\/4b4ujukr\/7weHKopaMkaPKRKJ5LrMDw5V\/PhJ6b9+tbhpk1aMMjGJSLkucOZ35qNPSD\/6WZQllZLc+qfpaVPvfdaer8W2\/YVapAIxScmh3VMU2P1XtoZG1X3k\/aKWiUkkc7v3m1fNf\/c1KwBs266eP2f64Y+Xil8qxkhmT+cnzecnTRrAOyHL8Zc5d1gdcuif3n8fOttsIi\/vf+CG4hWLMZBtvmcC+N4T73d9NVbU0jCp4OsRNGBPNGBPNLA8+uij+vcLR3743td6pFs+os7MLL3yq7m\/\/WvHqX+TFuZevevp31UfmYXzr54fMZkAwFRjuxEAun7+N\/9x9viuph0A8N7785956i\/Pzf6x\/ea21diXcibJee7sl78UO\/1\/YDIDaBqASQMwmTY\/YEzHT3zZ+6nG27zjP3\/UN+i7\/\/nG9fW\/Dp76+omHX9z1VOP6+lKV\/4NCknav6sGvgwk0TQMwmcAEJpO5viHV8rfXfRIAfnPhNQD4xdu+j264hSUVgyR5+Q13fkZy\/nnM77d+7m7bV7rNDQ2W+oYIwORc8I\/vnZ+cC07OBf\/4XvBTjbcBQOP6+u2ObYGZN6AJ3rj01j0fu7vku\/CBIPn5k\/Vzn4\/5\/WC1VtzaKj7cvL5h8\/qGTzUav3zPx+7+wagnED57YX4aw4spOEnavZj\/9cWf\/bv8Dwfef\/lk9H\/\/J+Mqbq\/75Pz7yr\/+\/qd16zZxo1ckjPGkKYry+AHbl79iu++rmjJ\/5bFvS84\/s2zcmGYVjevrP7rhlid\/f+xHn\/\/nYhb1A40xnpRv\/7124cINO+4BANt9u011dVf+6R8zruWej93tqN7IjV7xMMbTuseHxGuTLNcc+1mWK9ru2MaNXvEozPWI\/zz7Emd6RSVfT78Ontr4vSYA6Gr6UiHKwyRnJeONmNLD12FpwJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5o8P96Pr\/hNZvwcwAAAABJRU5ErkJggg==","height":388,"width":141}}
%---
%[output:0cabd540]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAI0AAAGECAIAAACERDSOAAAAB3RJTUUH6gQaESsZvtAfkQAAHAtJREFUeJztnX9sG9d9wL8UTyElnWOlHmVTluDU9tqo4mZMLhslUYEqqAo1ae02m6OkqQVoWtJBrYA5NZxGwQCvqJU1cK2u7FwkHuvOXZYyWdbEWQa33sIiUVs5mtUaISEHSVSrpkxFdCwpOlpiRN7tj6\/0fD7+Fn9IX+b7gWGQFO\/u3fvc973v3b3HM4XDYWDWPGWrXQAmI9gTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDdgTDa55crvdjji8Xm+yJRVF6erq6urqUhQllxL4\/f7m5ma3241vDx065Pf7AaCvr6+9vT0UCqVdFoua9stihfotpt5KJmVISygUam9vF1Uq9jQrJPGqu7u7u7s7FArt3bvXbre7XC5ZllMsKcvy8ePHV7BJA42NjUNDQ\/ja7Xa\/9NJLX\/rSlzJZ0Ov19vb27tu3r7u7GwD6+vq++MUvut3uxsbGzLfe39+\/4r9mgt\/v7+7ufvDBB7GQWGYAwLeZk6rdwwOhr68PN+BwOPSvT506JeIJP\/nBD36AR7fhkMG\/Ymj29fXpX7e3tw8ODuLR7fV6BwYGFEXp6OgQcXzgwAGHw9Hc3IxBJlAU5cSJE06ns6OjAz\/p7+8fGhoSknBDDocjdcTrIyZ+Ef1f9WEhdhC\/cOjQoWSFPHz4cENDgyhka2urz+cTkkQbpi+k+FBs2uv1pvJks9mamppGRkZCodDY2BgABINBRVHGxsbq6uoaGhoM35+cnBwaGtq1a9dzzz2nbyscDkddXd3p06cVRQkGgwAwNjaGr5uamm666SaxD\/v27ZNl2ePxtLa2AkAgEOjs7PR6vdXV1U8\/\/bR+W+Pj46Ojoy0tLQmDvq+vb2RkxOv1er3eYDCYSVh4vd6TJ0+6XC6PxzM6OurxePR\/xWamqanJ5\/O5XK6BgQFxJAUCgdtvv30FhcTj0uVy6QvpdruPHTvm8XiGhobsdvuBAwfQX5o8oq2tbWZmZmpq6p133gGA0dHR0dHRwcHBpqamDRs2GL68bds2\/B8XEZ+j72AwePnyZfQ0ODiIq2pra0uxdVmWa2pqKioq7HY7HiKpS4uEQqGRkZGmpiabzWaz2fbs2fPKK68YjvRk9Pb2Tk1NDQ0NGdoln88XCASwtE6n0+l0njhxAsuTYSH14YjRc\/r06bq6OofDIepnampqcHCwoaFhy5Ytsix3dnYODw8PDw9DWk81NTUA8MILL4yMjNx\/\/\/3V1dW\/\/e1vR0dHUYmBrVu3JlvPtm3bRkdHT548OTMz89BDDwWDwV\/+8pdi\/cmorq5O\/YWETE1NzczMJCxhCrD2AaC3tzc+gRobG0MfWRWypqamuroaD3GbzXbq1KmhoSHcytWrV4PBYCAQaG1tdTgcJ0+eDAaD7733XjAYtNvt8fGXxtOWLVsaGhpeffVVTdM+97nP2e32p556CgCam5szrQAA8f1XX321oaHhtttum5mZeeaZZ\/DAyWo9hoINDg7GH7\/62skcTIt8Pt++ffsA4Lvf\/a6+6d66dauiKPpGIhNSBFllZaXdbq+rq\/N6vT6fz+fznTp1qr6+Ptn303iSZbmlpWViYsJkMt18880tLS0AsIL6xWo9f\/683W5vaGjAvi1Zw50JolkQHUlfXx\/25PpuNRQKPffcc3feeWfaJNDtduPiHR0dTqfTbrdXVFSIv4ouFgCwLers7ExbeFmW9+\/fPzo6KjrI\/v5+bMcAoK2tLRAI+Hw+ka9hbY+Ojo6Pj4tECeNPSrqRZZqbm2VZxuYeWzYMzKxOm7AEw8PDbW1tsizb7XZI1E42NzcfO3aso6MDD+rUtLa2ejye7u7ugYEB3IRIyvv7+\/v6+jAZ2bVrVyZ5RHd39zvvvIOJGa5Kr8Fms\/30pz\/du3evw+EAAJfLhStPS2Nj40svvSQWBIC6uronnnhClmXMmzBNdzqdmEtjv4jFcDqd4uzIFA6HM9kes7rwdSMasCcasCcapM8jyBGNRvF\/SZIkqUR2kNhuoAN8ga8jkQgAxGIx\/D8ajZrNZgAwmUwAoGmaqqqSJGmaZjabJUkym82apgmFVFyurSKKUBAvsOpNJpN4UVZWFo1G5+fnq6qqysvLsZYtFouod\/EhrmFxcdGwzmg0urCwEI1Gw+FwRUVFWVkZLohGzWYzml5TLouXl8c7wHpHGRgQqAFfmEym8vJyuL6+8BMACAaD9fX1Vqt1xYVZWFgIBAKbNm2SJCmhS03TNE2D5aAEAIxFFGlQWGiX+Vl1Qgega46EBlVV5+fnKysrKyoqTCaTJElY18KBtEzqzeVYKbgJi8Uiy3KyVYl9gSRxubi4aAhKtAgAZWVl8QpzcZl+sWQO9NGAnYHoGCRJwmjASy96B9FoNMdQKBqiclN8B+\/O2Gw2q9Ua7xJFYlxiROJrfX+ZoUtpYmIiGo1u3LgRV72wsACJHBjioLy8HLsBvYMUmxHkHgprCtzfFEEJKeNS0zT8ZHFxUVGUqqqqsrIybGz1+Y4kSdKLL7742muv7d+\/3263izgQXbEIiPn5+cuXL5OIg1+fHfzRr\/75hmdv3VreeOXRFzv+9Ku3fiy7q\/v5JZe4xBeLi4tSbW3tG2+8YTKZtm\/fnsn21j7nZn93y67t0u3vH3\/6odnvzW28r251PWUCxozVak0WBmVLl82JONAjzqXi0VRV1WLFLEyhoacnE1RNNWlq6u+k0LwGWR1POdZR2ujPxFOGq1oj0ChltqiqatJUrfAbKlpQFttTcY5fVVOhiP1TEXaqZOMJNBMUIaCKxZKniYmJ1S1HflE1FVTTapcin0ibN29e7TLkH1VVtRIKJiDd7uG5ekJiWqwsnacUi69BCHtKgRrDeErjikpSDnQ9pa7imBYzaQBaWlO5UrSgXB1Phd69WCxWpgIUXFPxWAVPBW9tNE3VipdHFKfxpNrupSamxkzFuBxRPJY8BQKB1S1HfonFVFNpJealGU+qGivB6xG1tbWrXYyVkOIaqKqmvx7B9zVWGQ0gFlvKzEsGkvefIO35kxqDWPrre7mnaiV7X6MYaKBGS+y2e0mePwHE1JgWK9JZLp8\/rRw1pqrRUuqe6J8\/RZeZn5\/HtzMz0zGIqbGlq3uKooRCITEcce2PP0wIpXjSK7ly5QoA6Odyowmr1SpJUiyiqrGlE10cLI3fnJmZEV+enp62WCxWq7WiomItzMhIzRotXHyU4IBqBKtekqTq6mocuquPEllep87HYrGlds9qteL0ejEwG68CX7lyZXZ29oYbbtDLE6vCgcBrJ\/gkAFjdW7p6Jfox7ogYX22z2fAt1h2O7xW\/jWRAjamqqhryCIwY8WsDuBW73a4\/IBYWFhRFiUajBnliJL3hmCgaxYsnMRD+6tWrUR3XipJESdZooGlaJsPC9HMXDNsyyBMtp35Bq9V65coVs9m8sLBQaHkF8aQPC4wSvZL33ntPkqQbb7wR+wbIRUkSVFVTc7tenkKemPkUjUZnZ2dheUKRvoOEfOcs+TkhT6ZEHLCiuwYAWZaj0Sj2GQVC01RN0\/J+\/pQs3bDZbNjt4b7H5yz6bm9lOUsWC+gLkYmSZAWam5vLtpTZoqmgqThuueBnUWIKlP5DQ86ysLCA2pLJ03fJibeS8NP4KJmdnQ2FQrFYTMxYxgBPrWSV0DRNVQsQT5ljyFkEyXKWSCSC1VtVVZUwZ5GOHj06MTGhqur09DSuxZBuAYDVasXGatOmTannzhWTZNdANQ1UFfOI1VSVkGTd3tzcXCQSkWUZZxjG5yyS0+k8evSozWbDv6GS+CjBEBad5FpHTZ\/vRdfMDFRslm688cbq6mqh0JCzSE6nc\/fu3ZOTk\/X19atb3KxIXcWqpmma8fwp25WsLoauZHV+36jQt220nPPyDCna\/afS\/B0qTV3+jY5SYRU8Fby10XAEX0bzCXOnOI1nicZTZteNCFGanlQcEFtCppY8UZynlqIPx1+qWfHia5BSjCcNNDx\/Kr14KjK5H8viB94SkmH\/lHola4oyWO37hHnHXFGG0zU0DUADqcJcuG2V8vlTMY5iDTjfW+toGmigLUVT4SlO41mCnmBJFUAJzScsTU9LopJbopWUg\/BEdJxl4upeMlQysQRQsvGEyR6kuVO4lu9rGKDqKU0VF+VHCYrZeJbBKs0nLOhOasVK9ooG1XhKhQYg2j3xvmCU7H2Nwu+YCKfSCSkyHWkKxCg2HMI2PT0D2tLJLgAoc8r09DRQnlQDFD2hFZzVVFFRoWmaGMh2bYzU9WPC9GMc8QtlZWXT09P68T1rHAKexJBCDBfxxIJYLPaHP\/yhsrKysrJSlmUch4vj2pYMaQAAsizX19fjGDkc0RgOh+fm5iRJunz58szMjBhTt4rTMdJyzVMgEKirq1vFoiCGaTb6WAEAq9Vqs9mwZj\/60Y\/iEOiFZa7Fje5mrqIoFy9exNc4\/nDDhg1isLsYmipGFINuNOTaMbfKv0uQiRVIPqED5z\/hgviIinA4HFmILF2GBQCAcDgciUSqqqr040fFGvSrNQwqNphDx3pzhnlBBaWonkRFzM3Nvf766wAgSZLFYrFYLFVVVfqhuCkO4dRqN2zYsH79eszM8fP169dv3LgR610\/DF+Mj782hjtuUHG8OXx8WyQSwefEWCwW\/E6hk9gCzic0pGH6o+8jH\/lIJBLBfVtcXMS9xRqXlp+apT9mswo4q9V67TqsBlarVUw71K8Kh3GL5AKWh+0Lc1Hdc5Rw0\/g18Qwxs9msqqp4vHhC9\/kib0eB3gpcP6NWHKfJYkXfcL377rvRaDQSieCD7nB6SFYBB7A0pTD+44QRg5vGcMGny+nDBQAqKyurq6uTbTq1+3yZy27+k+FtfBoG10\/fTNuCQVzAWSwWs9lstVpjsVh8wGG06QMuIZkMhTVsHctfVVWFsaLfNAAoiiJJUjQatVqt8\/PzaVtLw7S1FFErFknd1WXhSVXVN998Ex+Xhjugnwsl0rBkLXVeAs4w4StxU6Mlvv+UogmVJClFuMDy3Lr4rSdMC6VE09ZE+XHO0+XLl\/EZaJFIxGKxKIqSesKSlHDkXsJdwmfu4RfMZjPugKiphHuYY8BdK+X1e27o3vUHrCRJCwvzgEOOAGD5EVjJ8vu0W8fv6yfWxycX+i8bpu8ZPi8vL8faQFvYugKAxWIpKyu7ePGiviavi9rh4eGjR4\/C8hl+NG6eWrJeIRqNiqNMURRxgJhMJnwKH7YYmQdc5iRsaubm5padzerbvbASnp2draqquummmzI8LFaw9fjjJrr81EB9hYjawKfX6Wc7wfIks4RRe+15AHfdddemTZt27NixadOmO+64o6WlJW1yjEeH+BBTuEgkgmXCzgafEZb3FChh74JVsHvnlxu3OULTU7EF9U\/KbZ+4+RNYGPz1nIRHa46I4w\/rRJIkRVFisZjIRwxHLXYc+uNV33WJHQwEAm+\/\/faFCxfOnj279PzciYmJF198EQCGh4eHh4fxq3V1dbfeeiv+\/8lPfjK+UkCXEBsCTh9tCfuhbOsode8imppkq9X3EPHlN\/ZwGRdJ5Av4GxOgaz\/iK0QUQJ816A+d2dnZycnJiYmJoaGhQCBw5swZWL47mOA5x9hjxWsT0bZz587PfOYzWbUhCasptTZcJOHE+rRT6jMpjz6Zji9SwitGKfYi7YGSsAwYLhcuXDh37tzk5OS5c+dgWYzT6XQ6nZs3b8YHE6Z\/HvXExMSlS5fQVrJoa27O7kGN8TuMnSo+AxYA8LHBudRCtiRsAKLRKD5M3NB2rbhIgUAgEAhgxJw5cwaHDwkxaAXFGMj6ueHJoi1bbfpGA6\/LYYMevf7cVpbl8vLyos3fFkG8sLCAFwZRFT6WGE9F8Np8hqXC5gv1xIvRR0xqcn2+e+ba9O1Y6kYjWfMCSU4ScyHDUhnOohKWat26dZIkCR8YMfi12trazZs319bWZi7GQK6eDKTQtnPnzurq6h07dtxxxx0Ql3ekQPQluWtLliXqxWQYu6JUb7\/99tmzZ8+dO3f58uWzZ8\/iX3MXYyDPngzotb3wwguXLl3Cz3Ps2+K1ge5cWK8thRh9dp5Vo4oRo0\/JELSSLzEGTOFwOBQK7d27F5vOffv2dXd3A4Db7R4YGAAAl8vV2tqa+5YwvHbv3p1736bHoA3FRCIRVVUtFsvCwoK4vrXi\/DuhGEPEDA8PJ0sBVozeiykcDvf19W3btq27u9vv93d3dz\/++OM1NTUHDx48evSoz+c7ceKEy+WK\/6me3MlLSmKIGP0JJl6ExJQkYbQlBOsltRh0k+v+p0Pv5bp2T1GU3t7ezs7OsbGxwcFBl8s1Pz\/f09Nz8ODBxsbGQhcr95Qk4dmlOE8y\/GgQhte6desmJychiRiMj6KJSYaiKNe1y+Pj44qiOByOsbExu90uyzLee52amiqCJ6yInp4efGvQ9vzzz+PnhpQk9eVU\/dm+zWbD4Dt\/\/jwAvPzyy\/Gdv9Pp7OnpyXsLliPj4+PXPIVCoYMHDx48eBDvkK46ybSJS1xIJo1ksqYMWYNi9KAXyev1tra2+v1+7JCEpGAwiA3FunXrampqVraNPCYjQtvExAQmVLW1tQmjTWgzXCiD5Yj5zne+g2tA5bt3785Xm+b1ent7e0GXjmVLX1\/fyZMnYbnGhBdTOBz2+\/1PPvlkf3+\/SBbEn3PJI\/KykgxJ1rcVNFc2EAqFsC8HAMNBvzL0XiRFUQ4fPjw8PCwaDTTZ3t7e2toqy7Lb7V5Z\/Q4NDeGNK4fDoSjK+Ph44Tq5hI1kkXt+n883NzdXU1NTUVEhy7LP58ulCTF4kWRZPn78ePz3uru7Vxa5eoqfjCCrlZvZ7Xb8BWkAGBsby8WTwUspzqspRQrraQXJSCgUam9vdzgcDofD7Xbjh263Gz\/xer15L6SiKF1dXbj+vr4+QzG6urr0J16pCQaD4gb31q1b81jIAnpqbm5WFGV+ft7n88myvGXLlkyWGhgY2LNnj8\/n83g8x44d83q9fr\/\/1KlTXq\/X5XKdOHEi81rLEI\/HY7fbfT6f1+sdGRnBgwOLMTQ0BAAiK0mNw+FYt27d1NSUOA3NYyELeFOnsbFxBclIf38\/vtiyZUtDQwMUPh8R3bDNZmtqagKAUCg0MjLS1tYmy3JLS8vp06cz6WlsNltPT09HRwcAuFyu\/J6GFvbmWy7JSPEvjoRCobfeeuuBBx6A6xtqbL0zOc5aW1t9Pl8hyrZG84jiXxxRFOXAgQM9PT1FS0qzYi168vv9PT09R48eFVWWl4sjKQiFQl1dXfv37xftGwYuvsZozvtGs2LNecKT8OPHj4tIWlk+kjmhUOjAgQP6wwI7qrGxMUVRBgcH29ra8rvFFVDY+7nZgjdW9PkVXhzB64SYj+S9XRKX1BC8NCfu0e3atUukNqtIFp6w6Hv27BGpQfwnTFq8Xu+jjz4qDjhxbzZ1SpldPPn9\/ocffvjIkSO4DTwrXAuHGy3cbjfehgWA3t7elpaWtAd6dv1TY2Pjvffee\/jwYUVR\/H7\/W2+9tW\/fvpWX98MKnmN5PB6PxyPepibr\/gnbum984xvPP\/98Z2dnXoa4fAjB5g4AMuxxs873bDbbI4888q1vfQsA1uw90LUPXm1paGjIMH1dSV6Od946OztX\/ayCLh6PR1EURVGw6UsLgd9jKT38fv+zzz575MgRAHj44Yebm5vTNn1r7jy35AmFQt\/85jfvvffexsZGfV6Wein2VGwOHDhgt9tFjocv0p7brK3rEUwyOJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5okKmnX\/y32V5Vqf\/3wyPlBS0Zo8cUDoezXeaHR8p\/8pT0X79a2LRJK0SZmHikbBc497uy409KP\/5ZhCUVk+z6p8lJU\/f9lq6vRXf8hVqgAjEJyaLdUxTY+1fWunrVdeyDgpaJiSd9u\/eb18r+7msWANixU714wfSjnywWvlSMkfSeLo6XXRw3aQDvBs0nX+HcYXXIon\/64ANob7GKvLz3wRsKVyzGQKb5ngng+09+0PHVaEFLwySDr0fQgD3RgD3RwPzYY4\/p388f+9H7X+uStn9MnZpafPVXs3\/71\/Yz\/yrNz7521zO\/qzo2AxdfuzhkMgGAqdp6IwB0\/Pxv\/uP8yT0NuwDg\/Q\/mPvv0X16Y+WPrzS2rsS+lTILz3Jn7vhw9+39gKgPQNACTBmAybXnQmI6fus\/z6frbPKM\/f8zb733ghfr1m38dOPP1U4+8tOfp+vWbi1X+DwsJ2r3Kh74OJtA0DcBkAhOYTGWb65Itf3vtpwDgN5deB4BfvOP9+IbtLKkQJMjLb7jzs5Ljz6M+n+Xzd1u\/0llWV2feXBcGGJ8N\/PH9i+OzgfHZwB\/fD3y6\/jYAqF+\/ead9h3\/qTWiAN6+8fc8tdxd9Fz4UJD5\/snz+C1GfDyyW8lubxYdb1tdtWV\/36Xrjl++55e4fDrv9ofOX5iYxvJi8k6Ddi\/reWPjZv8v\/cOiDV05H\/vd\/0q7i9tpPzX2g\/Mvv\/6123SZu9AqEMZ40RVGeOGS97yvW+7+qKXNXH\/+25Pgz88aNKVZRv37zxzdsf+r3J378hX8qZFE\/1BjjSfn232uXLt2w6x4AsN6\/11Rbe\/V7\/5h2Lffccre9aiM3eoXDGE\/rnhgQr02yXH3iZxmuaKd9Bzd6hSM\/1yP+8\/zLnOkVlFw9\/TpwZuP3GwCgo+HL+SgPk5iVjDdiig9fh6UBe6IBe6IBe6IBe6IBe6IBe6IBe6IBe6IBe6IBe6IBe6IBe6IBe6IBe6IBe6IBe6IBe6IBe6IBe6IBe6IBe6IBe6IBe6IBe6IBe6IBe6LB\/wPY8cL0lQu\/KgAAAABJRU5ErkJggg==","height":388,"width":141}}
%---
%[output:2d20d499]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAI0AAAGECAIAAACERDSOAAAAB3RJTUUH6gQaESsZvtAfkQAAG\/NJREFUeJztnX9sG9d9wL8kTyElnWOlHmVTP+DUztqq4mZMLhslUYEqqAo1ae02m6OmqQVoWtJBrYA5NZxGwQCvqJU18Kyu7FwkHuvObZYyWdbEWQa33sIiUVs5mtUaISEHSVSrpkxFdGwpOlpiRN7tj6\/1fD7+Fn9IX+b7gWGQFO\/u3fvc973v3b3HM0UiEWDWPObVLgCTFeyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBuyJBtc8eTweZwI+ny\/Vkoqi9PT09PT0KIqSTwkCgUBra6vH48G3Bw4cCAQCADAwMNDZ2RkOhzMui0XN+GWxQv0W028lmzJkJBwOd3Z2iioVe5oTknjV29vb29sbDod3797tcDjcbrcsy2mWlGX56NGjK9ikgebm5pGREXzt8XhefPHFL37xi9ks6PP5+vv79+zZ09vbCwADAwNf+MIXPB5Pc3Nz9lsfHBxc8V+zIRAI9Pb2PvDAA1hILDMA4NvsSdfu4YEwMDCAG3A6nfrXJ06cEPGEn3z\/+9\/Ho9twyOBfMTQHBgb0rzs7O4eHh\/Ho9vl8Q0NDiqJ0dXWJON63b5\/T6WxtbcUgEyiKcuzYMZfL1dXVhZ8MDg6OjIwISbghp9OZPuL1EZO4iP6v+rAQO4hfOHDgQKpCHjx4sKmpSRSyvb3d7\/cLSaIN0xdSfCg27fP50nmy2+0tLS1jY2PhcHhiYgIAQqGQoigTExMNDQ1NTU2G709PT4+MjOzYsePZZ5\/VtxVOp7OhoeHkyZOKooRCIQCYmJjA1y0tLTfddJPYhz179siy7PV629vbASAYDHZ3d\/t8vpqamqeeekq\/rcnJyfHx8ba2tqRBPzAwMDY25vP5fD5fKBTKJix8Pt\/x48fdbrfX6x0fH\/d6vfq\/YjPT0tLi9\/vdbvfQ0JA4koLB4O23376CQuJx6Xa79YX0eDxHjhzxer0jIyMOh2Pfvn3oL0Me0dHRMTs7OzMz8\/bbbwPA+Pj4+Pj48PBwS0vLhg0bDF\/eunUr\/o+LiM\/RdygUunjxInoaHh7GVXV0dKTZuizLtbW1lZWVDocDD5H0pUXC4fDY2FhLS4vdbrfb7bt27Xr55ZcNR3oq+vv7Z2ZmRkZGDO2S3+8PBoNYWpfL5XK5jh07huXJspD6cMToOXnyZENDg9PpFPUzMzMzPDzc1NS0efNmWZa7u7tHR0dHR0cho6fa2loAeP7558fGxu67776amprf\/va34+PjqMTAli1bUq1n69at4+Pjx48fn52dffDBB0Oh0C9\/+Uux\/lTU1NSk\/0JSZmZmZmdnk5YwDVj7ANDf35+YQE1MTKCPnApZW1tbU1ODh7jdbj9x4sTIyAhu5cqVK6FQKBgMtre3O53O48ePh0Khd999NxQKORyOxPjL4Gnz5s1NTU2vvPKKpmmf\/exnHQ7Hk08+CQCtra3ZVgCA+P4rr7zS1NR02223zc7OPv3003jg5LQeQ8GGh4cTj1997WQPpkV+v3\/Pnj0A8N3vflffdG\/ZskVRFH0jkQ1pgqyqqsrhcDQ0NPh8Pr\/f7\/f7T5w40djYmOr7GTzJstzW1jY1NWUymW6++ea2tjYAWEH9YrWePXvW4XA0NTVh35aq4c4G0SyIjmRgYAB7cn23Gg6Hn3322TvvvDNjEujxeHDxrq4ul8vlcDgqKyvFX0UXCwDYFnV3d2csvCzLe\/fuHR8fFx3k4OAgtmMA0NHREQwG\/X6\/yNewtsfHxycnJ0WihPEnpdzIMq2trbIsY3OPLRsGZk6nTViC0dHRjo4OWZYdDgckaydbW1uPHDnS1dWFB3V62tvbvV5vb2\/v0NAQbkIk5YODgwMDA5iM7NixI5s8ore39+2338bEDFel12C323\/yk5\/s3r3b6XQCgNvtxpVnpLm5+cUXXxQLAkBDQ8Pjjz8uyzLmTZimu1wuzKWxX8RiuFwucXZkikQi2WyPWV34uhEN2BMN2BMNMucR5IjFYvi\/JEmSVCY7SGw30AG+wNfRaBQA4vE4\/h+LxSwWCwCYTCYA0DRNVVVJkjRNs1gskiRZLBZN04RCKi7XVhFFKIgXWPUmk0m8MJvNsVhsYWGhurq6oqICa9lqtYp6Fx\/iGpaWlgzrjMVii4uLsVgsEolUVlaazWZcEI1aLBY0vaZcli4vT3SA9Y4yMCBQA74wmUwVFRVwfX3hJwAQCoUaGxttNtuKC7O4uBgMBjdt2iRJUlKXmqZpmgbLQQkAGIso0qCw2C4Ls+qkDkDXHAkNqqouLCxUVVVVVlaaTCZJkrCuhQNpmfSby7NScBNWq1WW5VSrEvsCKeJyaWnJEJRoEQDMZnOiwnxcZl4slQN9NGBnIDoGSZIwGvDSi95BLBbLMxRKhqjcNN\/BuzN2u91msyW6RJEYlxiR+FrfX2bpUpqamorFYhs3bsRVLy4uQjIHhjioqKjAbkDvIM1mBPmHwpoC9zdNUELauNQ0DT9ZWlpSFKW6utpsNmNjq893JEmSXnjhhVdffXXv3r0Oh0PEgeiKRUAsLCxcvHiRRBz8+vTwD3\/1Lzc8c+vWCue7j7zQ9af33\/qR3K7uF5Z84hJfLC0tSXV1da+\/\/rrJZLrllluy2d7a58zc75p3ftR8h\/Kjnz4wd3B+4331q+spGzBmbDZbqjAwX71sTsSBHnEu9UGgPK8baXA1pU4DLc2r4ynPOsoY\/RpoABk8ZbmqNUJ5xhMGVAm2U7KgLLWn0hy\/mqZmGU8FoQQ7VZ7xJC75lA1XPU1NTa1uOQqNppUwnkqAub6+frXLUHjUsgsoGtlOUvBcPSmqppo0dcWLr0EIe0pDXI2DZsrY8lFJyoGup\/RVHI\/HIUM4FYaSBeXqeCr27qmqir1T2XRRq+CpBK1NXI2btGwvSeRJaRpPqu1eemJqzKxayieahKdgMLi65Sgs8eV2r2wox3jSIK7G8T532SABQF1d3WoXYyWkvgaqqfG4Sc1wSYzWfY1yjKer509qGfZPJSb\/YznD+ROe5+a3kmwoWVCWbzypmT0RokzPn+JxiJtK0+zx+dMK0QDi8bgWX+1yFBTy50+xZRYWFvDt7OXZeFVcW+44FEUJh8NiOOLaH3+YFErxpFdy6dIlANDP5UYTNptNkqR4LK7Frl42wsHS+M3Z2Vnx5cuXL1utVpvNVllZuRZmZKRnjRYuMUpwQDWCVS9JUk1NDQ7d1UeJLMvxeVWNX+2ebDYbTq8XA7PxKvClS5fm5uZuuOEGvTyxKhwIvHaCTwKA1b2lq1eiH+OOiPHVdrsd32Ld4fhe8dtIBlQ1rsaN12ExYsSvDeBWHA6H\/oBYXFxUFCUWixnkiZH0hmOiZJQunsRA+CtXrsR0XCtKCiUrQFW1bK7w6ecuGLZlkCdaTv2CNpvt0qVLFotlcXGx2PKK4kkfFhgleiXvvvuuJEk33ngj9g2Qn5KkaKqqqnmNZEkjT8x8isVic3NzsDyhSN9BQqFzlsKckKdSIg5Y0V0DgCzLsVgM+4wioamaVoTrsKnSDbvdjt0e7ntizqLv9laWs+SwgL4Q2ShJVaD5+flcS5kbGqgYT1CKO4ViCpT+Q0POsri4iNpSydN3ycm3kvTTxCiZm5sLh8PxeFzMWMYAT69ktVBVTVXxOuzqXIs15CyCVDlLNBrF6q2urk6as0iHDx+emppSVfXy5cu4FkO6BQA2mw0bq02bNqWfO1dK0lwD1VRVyzQubFVI1e3Nz89Ho1FZlnGGYWLOIrlcrsOHD9vtdvwbKkmMEgxh0UmucVT12vlTKmJrZgYqNks33nhjTU2NUGjIWSSXy7Vz587p6enGxsbVLW5OpK1iTbva7mVQtUY8JcXQlZCc\/5QeTQO1OPleIiW7\/1Sm8zXyPn9aa6yCpxK0Nuq1xLzolKbxLM94UlVtbeZ7K6YcPWn4i0Rl1OqRnqeW\/vxJjWcYb1SyFKAglGM8LfdPq12KQkI1Lxc\/8JaULPPy9CtZU5hhte8TFhyLzbz8w3kAGkiVluJtq5zPn0pxFKuglWUeUW4s33MvgavSNJ7l6Em7+rs55TTjvQw9Xf1J1zJKykF4IjrOMlV1X53zWaKZn6WgDOMJINvfoVrL9zUMUPWUroqvl1S8iCpl42mGVZpPWOyd1MrrF46oxlN6SimpbO9rFH3HrmYQZRRNa3YeQE6IUWw4hO3y7GXQMOXTAECZn798+TJQnlQDFD2hFZzVVFlZqWmaGMh2deSbJBl+NEc\/xhG\/YzabL1++rB\/fs8Yh4EkMKcRwEU8siMfjf\/jDH6qqqqqqqmRZxnG4NputulrWn+jKstzY2Ihj5HBEYyQSmZ+flyTp4sWLs7OzYkzdKk7HyMg1T8FgsKGhYRWLghim2ehjBQBsNpvdbsea\/fCHP4xDoBeXEXGj6U5yFUU5f\/48rgTHH27YsEEMdhdDU8WIYtCNhlw75lb5dwmysQKpJ3Tg\/CdcEB9REYlEoovRa0mEBpFIJBqNVldX68ePijXoV2sYVGwwh4715gzzgopKST2Jipifn3\/ttdcAQJIkq9VqtVqrq6v1Q3HTHMLp1W7YsGH9+vXLeQQAwPr16zdu3Ij1rh+GL8bHXxvDnTCoONEcPr4tGo3ic2KsVit+p9hJbBHnExrSMP3R96EPfSgajeK+LS0t4d5ijUvLT83SH7M5BZzNZtN0ebnNZhPTDvWrwmHcIrmA5WH7wlxM9xwl3DR+TTxDzGKxqKoqHi+e1H2hKNhRoLcC18+oFcdpqljRN1zvvPNOLBaLRqP4oDucHpJTwAGkPHlKGjG4aQwXfLqcPlwAoKqqqqamJtWm07svlLnc5j8Z3iamYXD99M2MLRgkBJzVarVYLDabLR6PJwYcRps+4JKiZXEd1rB1LH91dTXGin7TAKAoiiRJsVjMZrMtLCxkbC0N09bSRK1YJH1Xl4MnVVXfeOMNfFwa7oB+LpRIw1K11AUJOMOEr+RNzXKyZzCVpgmVJClNuMDy3LrErSdNC6Vk09ZE+XHO08WLF\/EZaNFo1Gq1KoqSfsKSlHTkXtJdwmfu4RcsFgvugKippHuYZ8BdK+X1e27o3vUHrCRJi4sLcC2cNHwEVqr8PuPW8fv6ifWJyYX+y4bpe4bPKyoqsDbQFrauAGC1Ws1m8\/nz5\/U1eV3Ujo6OHj58GJbP8GMJ89RS9QqxWEwcZYqiiAPEZDLhr7xji5F9wGVP0qZmfn5+2dnc1dFGmgYAESUyNzdXXV190003ZXlYrGDricdNbPmpgfoKEbWBT6\/Tz3aC5UlmSaNWEsneXXfdtWnTpm3btm3atOmOO+5oa2vLmBzj0SE+xBQuGo1imbCzwWeEFTwFStq7YBXs3P6l5i3O8OxMfFH9kwr7x2\/+OBYGfz0n6dGaJ+L4wzqRJElRlHg8LvIRw1GLHYf+eNV3XWIHg8HgW2+9de7cudOnT199fu7U1NQLL7wAAKOjo6Ojo\/jVhoaGW2+9Ff\/\/xCc+kVgpoEuIDQGnj7ak\/VCudZS+dxFNTarV6nuIxPIbe7isiyTyBfyNCdC1H4kVIgqgzxr0h87c3Nz09PTU1NTIyEgwGDx16hQs3x1M8pxj7LEStYlo2759+6c\/\/emc2pCk1ZReGy6SdGJ9xin12ZRHn0wnFinpFaM0e5HxQElaBgyXc+fOnTlzZnp6+syZM7AsxuVyuVyu+vp6fDBh5udRT01NXbhwAW2lirbW1twe1Ji4w9ip4jNgAQAfG5xPLeRK0gYgFovhw8QNbdeKixQMBoPBIEbMqVOncPiQEINWUIyBnJ8bniractWmbzTwuhw26LHrz21lWa6oqCjZ\/G0RxIuLi3hhEFXhY4nxVASvzWdZKmy+UE+iGH3EpCff57tnr03fjqVvNFI1L5DiJDEfsiyV4SwqaanWrVsnSZLwgRGDX6urq6uvr6+rq8tejIF8PRlIo2379u01NTXbtm274447ICHvSIPoS\/LXlipL1IvJMnZFqd56663Tp0+fOXPm4sWLp0+fxr\/mL8ZAgT0Z0Gt7\/vnnL1y4gJ\/n2bclagPdubBeWxox+uw8p0YVI0afkiFopVBiDJgikUg4HN69ezc2nXv27Ont7QUAj8czNDQEAG63u729Pf8tYXjt3Lkz\/75Nj0EbiolGo6qqWq3WxcVFcX1rxfl3UjGGiBkdHU2VAqwYvRdTJBIZGBjYunVrb29vIBDo7e197LHHamtr9+\/ff\/jwYb\/ff+zYMbfbnfhTPflTkJTEEDH6E0y8CIkpSdJoSwrWS3ox6Cbf\/c+E3st17Z6iKP39\/d3d3RMTE8PDw263e2Fhoa+vb\/\/+\/c3NzcUuVv4pSdKzS3GeZPjRIAyvdevWTU9PQwoxGB8lE5MKRVGua5cnJycVRXE6nRMTEw6HQ5ZlvPc6MzNTAk9YEX19ffjWoO25557Dzw0pSfrLqfqzfbvdjsF39uxZAHjppZcSO3+Xy9XX11fwFixPJicnr3kKh8P79+\/fv38\/3iFddVJpE5e4kGwayVRNGbIGxehBL5LP52tvbw8EAtghCUmhUAgbinXr1tXW1q5sGwVMRoS2qakpTKjq6uqSRpvQZrhQBssR853vfAfXgMp37txZqDbN5\/P19\/eDLh3LlYGBgePHj8NyjQkvpkgkEggEnnjiicHBQZEsiD\/nk0cUZCVZkqpvK2qubCAcDmNfDgCGg35l6L1IiqIcPHhwdHRUNBposrOzs729XZZlj8ezsvodGRnBG1dOp1NRlMnJyeJ1ckkbyRL3\/H6\/f35+vra2trKyUpZlv9+fTxNi8CLJsnz06NHE7\/X29q4scvWUPhlBVis3czgc+AvSADAxMZGPJ4OX8pxXU34U19MKkpFwONzZ2el0Op1Op8fjwQ89Hg9+4vP5Cl5IRVF6enpw\/QMDA4Zi9PT06E+80hMKhcQN7i1bthSwkEX01NraqijKwsKC3++XZXnz5s3ZLDU0NLRr1y6\/3+\/1eo8cOeLz+QKBwIkTJ3w+n9vtPnbsWPa1liVer9fhcPj9fp\/PNzY2hgcHFmNkZAQARFaSHqfTuW7dupmZGXEaWsBCFvGmTnNz8wqSkcHBQXyxefPmpqYmKH4+Irphu93e0tICAOFweGxsrKOjQ5bltra2kydPZtPT2O32vr6+rq4uAHC73YU9DS3uzbd8kpHSXxwJh8Nvvvnm\/fffD9c31Nh6Z3Octbe3+\/3+YpRtjeYRpb84oijKvn37+vr6SpaU5sRa9BQIBPr6+g4fPiyqrCAXR9IQDod7enr27t0r2jcMXHyN0VzwjebEmvOEJ+FHjx4VkbSyfCR7wuHwvn379IcFdlQTExOKogwPD3d0dBR2iyuguPdzcwVvrOjzK7w4gtcJMR8peLskLqkheGlO3KPbsWOHSG1WkRw8YdF37dolUoPET5iM+Hy+Rx55RBxw4t5s+pQyt3gKBAIPPfTQoUOHcBt4VrgWDjdaeDwevA0LAP39\/W1tbRkP9Nz6p+bm5nvvvffgwYOKogQCgTfffHPPnj0rL+8HFTzH8nq9Xq9XvE1Pzv0TtnXf+MY3nnvuue7u7oIMcfkAgs0dAGTZ4+ac79nt9ocffvhb3\/oWAKzZe6BrH7za0tTUlGX6upK8HO+8dXd3r\/pZBV28Xq+iKIqiYNOXEQK\/x1J+BAKBZ5555tChQwDw0EMPtba2Zmz61tx5btkTDoe\/+c1v3nvvvc3Nzfq8LP1S7KnU7Nu3z+FwiBwPX2Q8t1lb1yOYVHA80YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90YA90SBbT7\/4b4ujukr\/7weHKopaMkaPKRKJ5LrMDw5V\/PhJ6b9+tbhpk1aMMjGJSLkucOZ35qNPSD\/6WZQllZLc+qfpaVPvfdaer8W2\/YVapAIxScmh3VMU2P1XtoZG1X3k\/aKWiUkkc7v3m1fNf\/c1KwBs266eP2f64Y+Xil8qxkhmT+cnzecnTRrAOyHL8Zc5d1gdcuif3n8fOttsIi\/vf+CG4hWLMZBtvmcC+N4T73d9NVbU0jCp4OsRNGBPNGBPNLA8+uij+vcLR3743td6pFs+os7MLL3yq7m\/\/WvHqX+TFuZevevp31UfmYXzr54fMZkAwFRjuxEAun7+N\/9x9viuph0A8N7785956i\/Pzf6x\/ea21diXcibJee7sl78UO\/1\/YDIDaBqASQMwmTY\/YEzHT3zZ+6nG27zjP3\/UN+i7\/\/nG9fW\/Dp76+omHX9z1VOP6+lKV\/4NCknav6sGvgwk0TQMwmcAEJpO5viHV8rfXfRIAfnPhNQD4xdu+j264hSUVgyR5+Q13fkZy\/nnM77d+7m7bV7rNDQ2W+oYIwORc8I\/vnZ+cC07OBf\/4XvBTjbcBQOP6+u2ObYGZN6AJ3rj01j0fu7vku\/CBIPn5k\/Vzn4\/5\/WC1VtzaKj7cvL5h8\/qGTzUav3zPx+7+wagnED57YX4aw4spOEnavZj\/9cWf\/bv8Dwfef\/lk9H\/\/J+Mqbq\/75Pz7yr\/+\/qd16zZxo1ckjPGkKYry+AHbl79iu++rmjJ\/5bFvS84\/s2zcmGYVjevrP7rhlid\/f+xHn\/\/nYhb1A40xnpRv\/7124cINO+4BANt9u011dVf+6R8zruWej93tqN7IjV7xMMbTuseHxGuTLNcc+1mWK9ru2MaNXvEozPWI\/zz7Emd6RSVfT78Ontr4vSYA6Gr6UiHKwyRnJeONmNLD12FpwJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5owJ5o8P96Pr\/hNZvwcwAAAABJRU5ErkJggg==","height":388,"width":141}}
%---
