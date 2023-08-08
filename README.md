# MPCforMMGship


## 使い方

- 初めて使用するときは、Pythonモジュールを予めインストールしておく。
  ```
  python -m pip install casadi
  python -m pip install do_mpc  
  python -m pip install pandas  
  python -m pip install numpy  
  ```
  - 上記の他に以下のモジュールも必要になることもある
    ```
    python -m pip install onnx
    python -m pip install asyncua
    ```

- ターミナル等で以下のコマンドでMPCのフォルダをクローンし、ディレクトリを移動する。
  ```
  git clone https://github.com/Keita058/MPCforMMGship.git
  cd MPCforMMGship
  ```
- pythonファイル内で各パラメータを調整した後に以下のコマンドでMPCを実行する
  ```
  python <python_file_name>
  ```


## 各フォルダ、ファイルについて

`input`:微係数の推定値が入ったcsvファイルや目標軌道の座標、その時の制御入力、速度のcsvファイルがある。  


`output`:MPCを実行した結果のcsvファイルが出力される。  


`fixed_coefficients.py`:固定の微係数を設定してMPCを実行する。船体主要目・微係数等のパラメータは[Introduction of MMG standard method for ship maneuvering predictions](https://link.springer.com/article/10.1007/s00773-014-0293-y)内の数値を参考に設定。  


`estimated_coefficients.py`:栗林手法によって推定された微係数を用いてMPCを実行する。168行付近のaverageをTrueにすることで推定値の平均値を用いてMPCを実行する。averageがFalseの場合はランダムに推定値が選ばれてMPCが実行される。  
