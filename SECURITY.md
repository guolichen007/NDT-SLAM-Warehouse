# 软件安全

## 漏洞报告

如发现本项目的安全漏洞，请通过以下方式私密报告：

1. GitHub Security Advisory：在仓库 Security 标签页提交私密报告
2. 直接联系仓库维护者

请勿在公开 Issue 中报告安全漏洞。

## 支持版本

| 版本 | 支持状态 |
|---|---|
| `master`（当前开发主线） | 支持 |

正式 release tag 发布后将列出具体支持版本。

## 依赖安全

本项目依赖：
- ROS Noetic
- PCL
- Eigen3
- Sophus
- g2o
- OpenCV
- yaml-cpp
- TBB

定期关注这些依赖的安全公告。GitHub Dependabot 已配置对 GitHub Actions 的监控。

## 敏感信息

- 仓库中不包含生产服务器的 IP、凭证、密钥
- 现场日志、bag、地图数据不在 Git 中
- `.gitignore` 已排除 `server_runs/`、`maps/`、`logs/`、`bag/`、`pcd/`

## 运行安全

运行时的安全行为（安全码、几何权限、障碍检测）见 [SAFETY.md](SAFETY.md)。
