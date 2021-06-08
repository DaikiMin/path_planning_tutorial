# Path Planing Tutorial
経路計画に関するTutorialです．

# Dynamic Window Approach Tutorial (DWA)

DWAは1997年に有名なロボティクスの教科書である『確率ロボティクス』の著者の人達が提案した
Local Path Planningアルゴリズムです．

非常に単純なアルゴリズムでありながらも，ロボットの運動モデルを考慮したパスを
高速に生成し，最終的なな制御入力まで計算できることから，多くのロボットで使用されています．

特に，動的環境を走行する屋内ロボットにおいては，広く利用されており，ROSのデフォルトパッケージである，
Navigationのbase_local_plannerもDWAを元にしたソフトになっています．

[詳しくはこちら](dwa_tutorial)

# Potential Method

障害物と目標位置の座標にポテンシャル関数と呼ばれる関数を定義し，その関数の勾配（座標成分ごとの偏微分）から進むべき方向を導出する手法  

[詳しくはこちら](potential_method)  
