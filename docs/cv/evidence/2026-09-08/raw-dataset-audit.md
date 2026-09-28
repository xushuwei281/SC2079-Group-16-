# MDP dataset audit

Source: `${CV_WORKSPACE}/data-original/20260908T142129Z`

Result: **FAIL**

not supplied; augmentation/session separation UNVERIFIED

| Split | Images | Valid labels | Empty labels | Objects |
|---|---:|---:|---:|---:|
| train | 6948 | 6948 | 102 | 6846 |
| valid | 790 | 790 | 12 | 778 |
| test | 344 | 344 | 7 | 337 |

## file_sha256 duplicates

Groups: 56; cross-split groups: 25; extra copies: 59.

## decoded_rgb_sha256 duplicates

Groups: 56; cross-split groups: 25; extra copies: 59.

## Errors

- Conflicting annotations for file_sha256 duplicate group: ['valid/images/20240205_194326_jpg.rf.f62b24840807063415c564aec4794737.jpg', 'test/images/20240205_194326_jpg.rf.31161a8a453f3e0bebf6dc686046bde5.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240205_194325_jpg.rf.af1e62e25546b34609acde4e6b07c936.jpg', 'train/images/20240205_194325_jpg.rf.f5aded7d00492d75a794575f22fcf5a0.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240206_202813_jpg.rf.da7ee55218525d080569279325b030ef.jpg', 'test/images/20240206_202813_jpg.rf.253e1ff66b41c18ca54784c472781655.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240206_202705-0-_jpg.rf.3b6f684fe0bf37444099ded8fc44d621.jpg', 'train/images/20240206_202705-0-_jpg.rf.a9d37e12fee0f75402eeb961beae06a8.jpg', 'train/images/20240206_202705-0-_jpg.rf.fbd0299f178e679498e4d0aa2133df02.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240206_120107_jpg.rf.7495003812c4563258d650343bb7e8b7.jpg', 'train/images/20240206_120107_jpg.rf.ea358aa204f2af7847968ff620f03e14.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240206_120113_jpg.rf.d7a7938bcf5f0b81c33c7c115cf72206.jpg', 'valid/images/20240206_120113_jpg.rf.ba3033b163541ee6cc38c62560199b51.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240206_202710_jpg.rf.c70ab17d267a894d0aa583b8f2fef90d.jpg', 'train/images/20240206_202710_jpg.rf.ef27d3863daf7c9933719d51ed3626dc.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240126_085013_jpg.rf.eaa15c125e078983b0e47e195cad9348.jpg', 'valid/images/20240126_085013_jpg.rf.aae1918febf26497ec39936156954c3e.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240205_194329_jpg.rf.38110c9aa3e291b835ee20dea9a3f924.jpg', 'train/images/20240205_194329_jpg.rf.e084976570df553abaecc3e40a50830e.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240126_085010_jpg.rf.7165c0a237d1d814a416ad8ff1f5be23.jpg', 'train/images/20240126_085010_jpg.rf.8c5aa5ba8abf3642856032bb304ea980.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240206_120108_jpg.rf.3b9066a6cfc0ed4170d35822c76c0a23.jpg', 'valid/images/20240206_120108_jpg.rf.5ebb4a3a64091101452dfd1fadc3ddbf.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240206_202724-0-_jpg.rf.becc2f9fe321b03ffe4288564bc3a484.jpg', 'valid/images/20240206_202724-0-_jpg.rf.4350bcd37d5f2ad0a9e43b763f77b015.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240206_202723_jpg.rf.7720e9333bd736614ef4f09cd8d29580.jpg', 'valid/images/20240206_202723_jpg.rf.c6ae246375e6a76e0d73ed65ae09c511.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240205_194326-0-_jpg.rf.2e1aae2790fb4d4d2b504e5c218eeb45.jpg', 'train/images/20240205_194326-0-_jpg.rf.9ca5a03e0010f18ef5c1a5f4b185f60d.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240126_085000_jpg.rf.5994cd927e076cfcc83ac0dbcb506be1.jpg', 'train/images/20240126_085000_jpg.rf.8e1d6848fdfad5760818b204de933531.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240205_194227_jpg.rf.cf3debc465b776bd363f8e7946e0cd04.jpg', 'test/images/20240205_194227_jpg.rf.2161f1ee84eadcc2aca2b54dc2c82a81.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240206_120058_jpg.rf.6327faaf5f0a7e2b987172f4f03e89e0.jpg', 'train/images/20240206_120058_jpg.rf.e0f2b1649759e66efbfb1a249e26e05f.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240206_120103_jpg.rf.3432f86f4ac543e84d5650e1015967fa.jpg', 'valid/images/20240206_120103_jpg.rf.d8475ea515cf49c14a9a7a5890576dfe.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240205_194217_jpg.rf.f86ee0de497e9e43e851b4d6b0332e82.jpg', 'test/images/20240205_194217_jpg.rf.e5ee4e5ad2e7b969aa8498096d850269.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240206_120105_jpg.rf.8f9f9a6740e6da8fa3ddc98a077d9e0b.jpg', 'train/images/20240206_120105_jpg.rf.f89a8a5b513becd6e883cd866c90ff24.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240205_194142-0-_jpg.rf.02c6d90943050e945b81aa0b03d246b2.jpg', 'train/images/20240205_194142-0-_jpg.rf.884930eed792140310054aca0ef7719c.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240126_084954_jpg.rf.2a0829cd95cd94df3113b1271cc190cc.jpg', 'train/images/20240126_084954_jpg.rf.3f8e28c3817a6eecbd4963e3cc9b4b8c.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['valid/images/20240205_194333_jpg.rf.09d6509b3354f792bd5fc227b7916ea6.jpg', 'test/images/20240205_194333_jpg.rf.4c178df917cda34375e56a300fd0d2df.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['valid/images/20240206_202746_jpg.rf.0017639939cd5adbc3d2caf5fe90c8ca.jpg', 'valid/images/20240206_202746_jpg.rf.ad922cc6650d5e37c71238855e8d8df0.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240126_085017_jpg.rf.e6f9809691dc2fb0966ead7f59178823.jpg', 'test/images/20240126_085017_jpg.rf.42e0cfb1a3770fe0bdf7b9f1a9a2ceff.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240206_202717_jpg.rf.1bdd94db7571265157cec9921dc0036e.jpg', 'valid/images/20240206_202717_jpg.rf.bf57016c6a9eb39e8c8a54d00ae7913d.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240126_085011_jpg.rf.872acc7d60303576de32e4280633f344.jpg', 'train/images/20240126_085011_jpg.rf.c2ca35187b7cb09dbeff2ed1d3617ba6.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240205_194207-0-_jpg.rf.4c1cc211db4eb0383d040076af3f9d5b.jpg', 'train/images/20240205_194207-0-_jpg.rf.e54be6c910cc04d96d2ac2bd24bdc5a3.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240205_194208_jpg.rf.66187684cbd5939f27e2cc5721245157.jpg', 'valid/images/20240205_194208_jpg.rf.6b0aeda1d9b9a92437cc64bdf39b6d47.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240206_202722_jpg.rf.aa2bff665b34e8f3d2f4580dff7cf55e.jpg', 'valid/images/20240206_202722_jpg.rf.d507d2fcced5b39a54b73d07e5001090.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240126_085007_jpg.rf.7f01e7ea6c077b7c11e62f35a33ee04c.jpg', 'valid/images/20240126_085007_jpg.rf.d3aaa8821c4052ac471c8d65405bef55.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240205_194142_jpg.rf.4f3e125d7f76df0ba3e1ce2c34904445.jpg', 'train/images/20240205_194142_jpg.rf.d5dacc917a593e8c5d4d6084c1f4ad45.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['valid/images/20240206_120106_jpg.rf.23ad2de6572974602e205e6da1120753.jpg', 'test/images/20240206_120106_jpg.rf.99886d38fd6d6a0b604b34455f8edb38.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240205_194139_jpg.rf.5089fe99199d00963aaf2523ef701e8d.jpg', 'valid/images/20240205_194139_jpg.rf.9db97e098d50d0f3c2e91bff333f352f.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240126_084939_jpg.rf.eee0905f97999620abb275134451ee48.jpg', 'test/images/20240126_084939_jpg.rf.4e57b3c851804fb291c7ca730ca4d07c.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['valid/images/20240205_194134_jpg.rf.004c7189a9fc168ed145838f4badd04e.jpg', 'valid/images/20240205_194134_jpg.rf.c9bdc2bb442b7cd9d1392bead8e8c07c.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240206_202803_jpg.rf.6fb9bc61bcbdd563b127a60e4f332aa9.jpg', 'train/images/20240206_202803_jpg.rf.f26874e20e7aa1b1e7b4a62600e46b77.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240126_084950_jpg.rf.030c2274f884ea4e3deea6464f0f82ef.jpg', 'test/images/20240126_084950_jpg.rf.cac8c618df34cb997e9ce887985b001d.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240126_084952_jpg.rf.8973fd16c497acc3d6bffc858112a046.jpg', 'train/images/20240126_084952_jpg.rf.8c197355dfd7d80c45c3efedef4c3589.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['valid/images/20240206_202705_jpg.rf.8e8d5153d3c33e0f14c937c634bdd92a.jpg', 'test/images/20240206_202705_jpg.rf.2ca088a3c506992dd2e04454aa580759.jpg', 'test/images/20240206_202705_jpg.rf.303a02e27bb260e09f620c388d5b52f1.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240205_194136_jpg.rf.8f9d091e2d597c3c910610b9579ed2a1.jpg', 'train/images/20240205_194136_jpg.rf.ba01d01f3952c3d4d78c25956f8a3c86.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240206_202741_jpg.rf.14245cb60478b1202782477d4ac2a9db.jpg', 'train/images/20240206_202741_jpg.rf.51363a1c632388e04513eb3033813c3f.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240126_084937_jpg.rf.91443261fd28f54e6f26c22b7069378c.jpg', 'train/images/20240126_084937_jpg.rf.96cd8f2ae00429afd3072ffdd4a1cc63.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240206_202701_jpg.rf.8f407e3b2f690c33b8c6360a39b7c86b.jpg', 'train/images/20240206_202701_jpg.rf.aee0521867ba1799b02a8b8a31f2051d.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240126_085015_jpg.rf.55b28c9dee136b562fe3c6a52bd65f36.jpg', 'test/images/20240126_085015_jpg.rf.bae8521eddf633b72d1f59ab107552e5.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240205_194336_jpg.rf.484ac3df44f47914af5a58f8c90786b1.jpg', 'train/images/20240205_194336_jpg.rf.4c00d444dc558b0048215215e4e15bec.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240206_120059_jpg.rf.79ca542edde99995ef23f89b4212b770.jpg', 'train/images/20240206_120059_jpg.rf.e56868af8e7a79aac45278375d7077ec.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240206_120104_jpg.rf.426df5388a14d73464a1aefef57c7ba0.jpg', 'train/images/20240206_120104_jpg.rf.dd0d4fc1ab8746da027cbb6063d63086.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240206_202708_jpg.rf.21932224936c2db98f099544f5e557d8.jpg', 'train/images/20240206_202708_jpg.rf.9adaef4a64cc0ae0b92fd08f55356a94.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240206_202748_jpg.rf.0d37dc6dd9451a4799c88f63e46b00d7.jpg', 'valid/images/20240206_202748_jpg.rf.91b6a2a711a105e53a3dbd31d8a5b593.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240206_202704_jpg.rf.7be15f6eaf7990374f45c87ed1fdefe6.jpg', 'train/images/20240206_202704_jpg.rf.7d0991da89721143f409d5529230a46e.jpg', 'train/images/20240206_202704_jpg.rf.cd60c93efaed997678020660c8dc59b9.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240206_120100_jpg.rf.4ad853ce61b08d75a92bf79721977e95.jpg', 'train/images/20240206_120100_jpg.rf.92b68c82671ffd6645433ac9501cae89.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240205_194209_jpg.rf.1dc57f21f72836cb50945dfb9059a56f.jpg', 'valid/images/20240205_194209_jpg.rf.a0379cba469408d6e88acf3c6890f635.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240206_202820_jpg.rf.5552c2716a02b591cdc42c320e86b927.jpg', 'train/images/20240206_202820_jpg.rf.a66cbda738cc99ea78ffbe6e51208bec.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240206_120102_jpg.rf.702a2c6b7c90297b2a1268bb117989b3.jpg', 'test/images/20240206_120102_jpg.rf.a6a28a60646bf069c97c56b036f57382.jpg']
- Conflicting annotations for file_sha256 duplicate group: ['train/images/20240205_194135_jpg.rf.1aeb30b47a880f326bc4c66154735e5a.jpg', 'train/images/20240205_194135_jpg.rf.d9ccfb7b305c0467656e8c578454ce0f.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240206_202708_jpg.rf.21932224936c2db98f099544f5e557d8.jpg', 'train/images/20240206_202708_jpg.rf.9adaef4a64cc0ae0b92fd08f55356a94.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240206_202717_jpg.rf.1bdd94db7571265157cec9921dc0036e.jpg', 'valid/images/20240206_202717_jpg.rf.bf57016c6a9eb39e8c8a54d00ae7913d.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240206_202748_jpg.rf.0d37dc6dd9451a4799c88f63e46b00d7.jpg', 'valid/images/20240206_202748_jpg.rf.91b6a2a711a105e53a3dbd31d8a5b593.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['valid/images/20240206_202746_jpg.rf.0017639939cd5adbc3d2caf5fe90c8ca.jpg', 'valid/images/20240206_202746_jpg.rf.ad922cc6650d5e37c71238855e8d8df0.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240205_194325_jpg.rf.af1e62e25546b34609acde4e6b07c936.jpg', 'train/images/20240205_194325_jpg.rf.f5aded7d00492d75a794575f22fcf5a0.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240126_085011_jpg.rf.872acc7d60303576de32e4280633f344.jpg', 'train/images/20240126_085011_jpg.rf.c2ca35187b7cb09dbeff2ed1d3617ba6.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['valid/images/20240205_194333_jpg.rf.09d6509b3354f792bd5fc227b7916ea6.jpg', 'test/images/20240205_194333_jpg.rf.4c178df917cda34375e56a300fd0d2df.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240206_202710_jpg.rf.c70ab17d267a894d0aa583b8f2fef90d.jpg', 'train/images/20240206_202710_jpg.rf.ef27d3863daf7c9933719d51ed3626dc.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240206_202724-0-_jpg.rf.becc2f9fe321b03ffe4288564bc3a484.jpg', 'valid/images/20240206_202724-0-_jpg.rf.4350bcd37d5f2ad0a9e43b763f77b015.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240206_202803_jpg.rf.6fb9bc61bcbdd563b127a60e4f332aa9.jpg', 'train/images/20240206_202803_jpg.rf.f26874e20e7aa1b1e7b4a62600e46b77.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240205_194139_jpg.rf.5089fe99199d00963aaf2523ef701e8d.jpg', 'valid/images/20240205_194139_jpg.rf.9db97e098d50d0f3c2e91bff333f352f.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240126_085000_jpg.rf.5994cd927e076cfcc83ac0dbcb506be1.jpg', 'train/images/20240126_085000_jpg.rf.8e1d6848fdfad5760818b204de933531.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240205_194208_jpg.rf.66187684cbd5939f27e2cc5721245157.jpg', 'valid/images/20240205_194208_jpg.rf.6b0aeda1d9b9a92437cc64bdf39b6d47.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240126_084954_jpg.rf.2a0829cd95cd94df3113b1271cc190cc.jpg', 'train/images/20240126_084954_jpg.rf.3f8e28c3817a6eecbd4963e3cc9b4b8c.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240206_120108_jpg.rf.3b9066a6cfc0ed4170d35822c76c0a23.jpg', 'valid/images/20240206_120108_jpg.rf.5ebb4a3a64091101452dfd1fadc3ddbf.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240206_202820_jpg.rf.5552c2716a02b591cdc42c320e86b927.jpg', 'train/images/20240206_202820_jpg.rf.a66cbda738cc99ea78ffbe6e51208bec.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240126_085013_jpg.rf.eaa15c125e078983b0e47e195cad9348.jpg', 'valid/images/20240126_085013_jpg.rf.aae1918febf26497ec39936156954c3e.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['valid/images/20240205_194134_jpg.rf.004c7189a9fc168ed145838f4badd04e.jpg', 'valid/images/20240205_194134_jpg.rf.c9bdc2bb442b7cd9d1392bead8e8c07c.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240206_202741_jpg.rf.14245cb60478b1202782477d4ac2a9db.jpg', 'train/images/20240206_202741_jpg.rf.51363a1c632388e04513eb3033813c3f.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240205_194217_jpg.rf.f86ee0de497e9e43e851b4d6b0332e82.jpg', 'test/images/20240205_194217_jpg.rf.e5ee4e5ad2e7b969aa8498096d850269.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240206_202723_jpg.rf.7720e9333bd736614ef4f09cd8d29580.jpg', 'valid/images/20240206_202723_jpg.rf.c6ae246375e6a76e0d73ed65ae09c511.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240206_120105_jpg.rf.8f9f9a6740e6da8fa3ddc98a077d9e0b.jpg', 'train/images/20240206_120105_jpg.rf.f89a8a5b513becd6e883cd866c90ff24.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240126_084950_jpg.rf.030c2274f884ea4e3deea6464f0f82ef.jpg', 'test/images/20240126_084950_jpg.rf.cac8c618df34cb997e9ce887985b001d.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240206_202722_jpg.rf.aa2bff665b34e8f3d2f4580dff7cf55e.jpg', 'valid/images/20240206_202722_jpg.rf.d507d2fcced5b39a54b73d07e5001090.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240206_120104_jpg.rf.426df5388a14d73464a1aefef57c7ba0.jpg', 'train/images/20240206_120104_jpg.rf.dd0d4fc1ab8746da027cbb6063d63086.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240206_120059_jpg.rf.79ca542edde99995ef23f89b4212b770.jpg', 'train/images/20240206_120059_jpg.rf.e56868af8e7a79aac45278375d7077ec.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240206_202704_jpg.rf.7be15f6eaf7990374f45c87ed1fdefe6.jpg', 'train/images/20240206_202704_jpg.rf.7d0991da89721143f409d5529230a46e.jpg', 'train/images/20240206_202704_jpg.rf.cd60c93efaed997678020660c8dc59b9.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240206_202701_jpg.rf.8f407e3b2f690c33b8c6360a39b7c86b.jpg', 'train/images/20240206_202701_jpg.rf.aee0521867ba1799b02a8b8a31f2051d.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240206_202813_jpg.rf.da7ee55218525d080569279325b030ef.jpg', 'test/images/20240206_202813_jpg.rf.253e1ff66b41c18ca54784c472781655.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240126_085010_jpg.rf.7165c0a237d1d814a416ad8ff1f5be23.jpg', 'train/images/20240126_085010_jpg.rf.8c5aa5ba8abf3642856032bb304ea980.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240205_194329_jpg.rf.38110c9aa3e291b835ee20dea9a3f924.jpg', 'train/images/20240205_194329_jpg.rf.e084976570df553abaecc3e40a50830e.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240205_194227_jpg.rf.cf3debc465b776bd363f8e7946e0cd04.jpg', 'test/images/20240205_194227_jpg.rf.2161f1ee84eadcc2aca2b54dc2c82a81.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240126_085015_jpg.rf.55b28c9dee136b562fe3c6a52bd65f36.jpg', 'test/images/20240126_085015_jpg.rf.bae8521eddf633b72d1f59ab107552e5.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240205_194326-0-_jpg.rf.2e1aae2790fb4d4d2b504e5c218eeb45.jpg', 'train/images/20240205_194326-0-_jpg.rf.9ca5a03e0010f18ef5c1a5f4b185f60d.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240205_194207-0-_jpg.rf.4c1cc211db4eb0383d040076af3f9d5b.jpg', 'train/images/20240205_194207-0-_jpg.rf.e54be6c910cc04d96d2ac2bd24bdc5a3.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240206_120100_jpg.rf.4ad853ce61b08d75a92bf79721977e95.jpg', 'train/images/20240206_120100_jpg.rf.92b68c82671ffd6645433ac9501cae89.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['valid/images/20240206_202705_jpg.rf.8e8d5153d3c33e0f14c937c634bdd92a.jpg', 'test/images/20240206_202705_jpg.rf.2ca088a3c506992dd2e04454aa580759.jpg', 'test/images/20240206_202705_jpg.rf.303a02e27bb260e09f620c388d5b52f1.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240205_194142_jpg.rf.4f3e125d7f76df0ba3e1ce2c34904445.jpg', 'train/images/20240205_194142_jpg.rf.d5dacc917a593e8c5d4d6084c1f4ad45.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240206_120103_jpg.rf.3432f86f4ac543e84d5650e1015967fa.jpg', 'valid/images/20240206_120103_jpg.rf.d8475ea515cf49c14a9a7a5890576dfe.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240126_084939_jpg.rf.eee0905f97999620abb275134451ee48.jpg', 'test/images/20240126_084939_jpg.rf.4e57b3c851804fb291c7ca730ca4d07c.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240126_084937_jpg.rf.91443261fd28f54e6f26c22b7069378c.jpg', 'train/images/20240126_084937_jpg.rf.96cd8f2ae00429afd3072ffdd4a1cc63.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240205_194142-0-_jpg.rf.02c6d90943050e945b81aa0b03d246b2.jpg', 'train/images/20240205_194142-0-_jpg.rf.884930eed792140310054aca0ef7719c.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['valid/images/20240205_194326_jpg.rf.f62b24840807063415c564aec4794737.jpg', 'test/images/20240205_194326_jpg.rf.31161a8a453f3e0bebf6dc686046bde5.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240206_120102_jpg.rf.702a2c6b7c90297b2a1268bb117989b3.jpg', 'test/images/20240206_120102_jpg.rf.a6a28a60646bf069c97c56b036f57382.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240126_085007_jpg.rf.7f01e7ea6c077b7c11e62f35a33ee04c.jpg', 'valid/images/20240126_085007_jpg.rf.d3aaa8821c4052ac471c8d65405bef55.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['valid/images/20240206_120106_jpg.rf.23ad2de6572974602e205e6da1120753.jpg', 'test/images/20240206_120106_jpg.rf.99886d38fd6d6a0b604b34455f8edb38.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240206_120058_jpg.rf.6327faaf5f0a7e2b987172f4f03e89e0.jpg', 'train/images/20240206_120058_jpg.rf.e0f2b1649759e66efbfb1a249e26e05f.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240205_194135_jpg.rf.1aeb30b47a880f326bc4c66154735e5a.jpg', 'train/images/20240205_194135_jpg.rf.d9ccfb7b305c0467656e8c578454ce0f.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240206_120107_jpg.rf.7495003812c4563258d650343bb7e8b7.jpg', 'train/images/20240206_120107_jpg.rf.ea358aa204f2af7847968ff620f03e14.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240126_084952_jpg.rf.8973fd16c497acc3d6bffc858112a046.jpg', 'train/images/20240126_084952_jpg.rf.8c197355dfd7d80c45c3efedef4c3589.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240205_194336_jpg.rf.484ac3df44f47914af5a58f8c90786b1.jpg', 'train/images/20240205_194336_jpg.rf.4c00d444dc558b0048215215e4e15bec.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240206_202705-0-_jpg.rf.3b6f684fe0bf37444099ded8fc44d621.jpg', 'train/images/20240206_202705-0-_jpg.rf.a9d37e12fee0f75402eeb961beae06a8.jpg', 'train/images/20240206_202705-0-_jpg.rf.fbd0299f178e679498e4d0aa2133df02.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240205_194209_jpg.rf.1dc57f21f72836cb50945dfb9059a56f.jpg', 'valid/images/20240205_194209_jpg.rf.a0379cba469408d6e88acf3c6890f635.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240205_194136_jpg.rf.8f9d091e2d597c3c910610b9579ed2a1.jpg', 'train/images/20240205_194136_jpg.rf.ba01d01f3952c3d4d78c25956f8a3c86.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240126_085017_jpg.rf.e6f9809691dc2fb0966ead7f59178823.jpg', 'test/images/20240126_085017_jpg.rf.42e0cfb1a3770fe0bdf7b9f1a9a2ceff.jpg']
- Conflicting annotations for decoded_rgb_sha256 duplicate group: ['train/images/20240206_120113_jpg.rf.d7a7938bcf5f0b81c33c7c115cf72206.jpg', 'valid/images/20240206_120113_jpg.rf.ba3033b163541ee6cc38c62560199b51.jpg']

## Warnings

None.

## Limits and manual review

- Exact-file and decoded-pixel hashes cannot identify general augmentations, near duplicates, or adjacent video frames.
- Without complete, independently verified original/session groups, split independence remains unverified.
- A groups CSV enforces only the provenance supplied by its author; this tool cannot verify those group assignments.
- Empty labels are accepted as background negatives; inspect their images to confirm there are no unlabeled targets.
- Polygon checks cover syntax, finite normalized coordinates, nonzero bounds and area; they do not prove annotation semantics or absence of self-intersection.
- Decode and hash auditing does not establish that classes, target direction, or annotations are visually correct.
- Freeze and hash the resulting snapshot before training. Audit a stable local copy, not a source being changed concurrently.

Per-image hashes, annotation digests, duplicate membership, and per-class counts are in the JSON manifest.
