#!/usr/bin/env python3
"""检查文档合同：链接有效性、README 禁止项、中文规范、过期引用、索引一致性。"""

import os
import re
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# 技术标识符允许列表（中文检查时不视为违规）
TECH_ALLOWLIST = {
    'ROS', 'NDT', 'EKF', 'SLAM', 'API', 'CI', 'GitHub', 'Git',
    'Formal', 'Degraded', 'MapCommit', 'CargoSafetyStatus',
    'Code', 'Topic', 'Service', 'Tag', 'SHA', 'TF',
    'LiDAR', 'PCL', 'Sophus', 'g2o', 'TBB', 'OpenCV',
    'NDT-SLAM', 'NDT-SLAM-Warehouse', 'RViz', 'Noetic',
    'OBB', 'EKF', 'Gravity', 'Marker', 'Schema',
    'Cargo', 'Heartbeat', 'Manifest', 'Bundle',
    'PASS', 'FAIL', 'NOT_RUN', 'KNOWN_BASELINE_FAILURE',
    'CLEAR', 'NEAR_3M', 'NEAR_5M', 'LOST_HOLD',
    'EMPTY', 'CANDIDATE', 'LOCKED',
}


def find_md_files(root, exclude_dirs=None):
    """递归查找 Markdown 文件。"""
    if exclude_dirs is None:
        exclude_dirs = {'.git', 'build', 'devel', 'install', 'logs', 'maps',
                        'server_runs', '3rdparty', 'tbb', '.catkin_tools'}
    md_files = []
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames if d not in exclude_dirs]
        for f in filenames:
            if f.endswith('.md') or f.endswith('.markdown'):
                md_files.append(os.path.join(dirpath, f))
    return md_files


def check_markdown_links(md_files):
    """检查 Markdown 内部相对链接目标是否存在。"""
    errors = []
    for md_file in md_files:
        md_dir = os.path.dirname(md_file)
        with open(md_file, 'r', encoding='utf-8', errors='replace') as f:
            content = f.read()
        # 匹配 Markdown 链接 [text](path)
        links = re.findall(r'\[([^\]]*)\]\(([^)]+)\)', content)
        for text, link in links:
            # 跳过外部链接和锚点
            if link.startswith('http://') or link.startswith('https://'):
                continue
            if link.startswith('#'):
                continue
            if link.startswith('mailto:'):
                continue
            # 去掉锚点部分
            link_path = link.split('#')[0]
            if not link_path:
                continue
            target = os.path.normpath(os.path.join(md_dir, link_path))
            if not os.path.exists(target):
                errors.append(
                    f"{md_file}: 链接目标不存在 [{text}]({link}) -> {target}")
    return errors


def check_readme_forbidden(readme_path):
    """检查 README 不应包含的内容（作为标题或章节，导航链接中的引用除外）。"""
    if not os.path.exists(readme_path):
        return ["README.md 不存在"]
    with open(readme_path, 'r', encoding='utf-8', errors='replace') as f:
        lines = f.readlines()
    # 只检查标题行（以 # 开头）和非链接行中的禁止词
    forbidden = [
        'Known Limitations',
        'Not yet',
        'TODO',
        'NOT_RUN',
        '待完成',
        '待验证',
        '待开发',
        '已知问题',
        '后续优化',
        '下一步',
    ]
    errors = []
    for lineno, line in enumerate(lines, 1):
        stripped = line.strip()
        # 跳过硬编码技术术语（如 ROS, NDT 等）
        for term in forbidden:
            if term in stripped:
                # 如果在链接中 [text](url)，允许
                if re.search(r'\[.*' + re.escape(term) + r'.*\]\(', stripped):
                    continue
                # 如果是 Markdown 标题（以 # 开头），不允许
                if stripped.startswith('#'):
                    errors.append(
                        f"{readme_path}:{lineno}: README 标题含禁止项: '{term}'")
                # 如果不在链接中
                elif '](' not in stripped:
                    errors.append(
                        f"{readme_path}:{lineno}: README 含禁止项: '{term}'")
    return errors


def check_legacy_references(doc_files):
    """检查技术文档是否引用已删除的 legacy 组件。"""
    legacy_patterns = [
        (r'include/lidar_slam2/OdometryNode\.hpp', 'OdometryNode.hpp'),
        (r'include/lidar_slam2/MappingNode\.hpp', 'MappingNode.hpp'),
        (r'include/lidar_slam2/Visualizer\.hpp', 'Visualizer.hpp'),
        (r'include/lidar_slam2/SlamNode\.hpp', 'SlamNode.hpp'),
        (r'KISS-ICP', 'KISS-ICP'),
    ]
    errors = []
    for doc_file in doc_files:
        with open(doc_file, 'r', encoding='utf-8', errors='replace') as f:
            content = f.read()
        for pattern, name in legacy_patterns:
            if re.search(pattern, content):
                errors.append(
                    f"{doc_file}: 引用已删除组件 '{name}'")
    return errors


def check_roadmap_not_in_tech_docs():
    """确认 roadmap.md 不在 src/ndt_slam/doc/ 中。"""
    tech_doc_dir = os.path.join(REPO_ROOT, 'src', 'ndt_slam', 'doc')
    roadmap_path = os.path.join(tech_doc_dir, 'roadmap.md')
    if os.path.exists(roadmap_path):
        return ["roadmap.md 不应存在于 src/ndt_slam/doc/（已移至 docs/project/）"]
    return []


def check_canonical_docs_language(tech_doc_dir):
    """检查技术文档的标题和说明是否为中文。允许技术标识符出现英文。"""
    # 应该中文化的英文模式（Markdown 标题中出现）
    english_title_patterns = [
        (r'^# (Contributing|Branch Discipline|Purpose|Commit Style|'
         r'Safety-contract PR Requirements|Verification Checklist|'
         r'Server Validation Evidence|Testing & Acceptance|Troubleshooting|'
         r'Known Limitations|Not yet|TODO)',
         '英文标题'),
    ]
    errors = []
    if not os.path.isdir(tech_doc_dir):
        return errors
    for fname in os.listdir(tech_doc_dir):
        if not fname.endswith('.md'):
            continue
        fpath = os.path.join(tech_doc_dir, fname)
        with open(fpath, 'r', encoding='utf-8', errors='replace') as f:
            for lineno, line in enumerate(f, 1):
                for pattern, desc in english_title_patterns:
                    if re.match(pattern, line.strip()):
                        errors.append(
                            f"{fpath}:{lineno}: {desc}: {line.strip()[:60]}")
    return errors


def check_docs_index_links(index_dir):
    """检查 docs 索引链接的目标文件存在。"""
    index_path = os.path.join(index_dir, 'README.md')
    if not os.path.exists(index_path):
        return []
    with open(index_path, 'r', encoding='utf-8', errors='replace') as f:
        content = f.read()
    links = re.findall(r'\[([^\]]*)\]\(([^)]+)\)', content)
    errors = []
    for text, link in links:
        if link.startswith('http') or link.startswith('#'):
            continue
        link_path = link.split('#')[0]
        if not link_path:
            continue
        target = os.path.normpath(os.path.join(index_dir, link_path))
        if not os.path.exists(target):
            errors.append(
                f"{index_path}: 索引链接目标不存在 [{text}]({link}) -> {target}")
    return errors


STALE_CURRENT_PATTERNS = [
    (r'\b944\b.*\btests\b', '944 tests 过期状态'),
    (r'14\s*failures', '14 failures 过期状态'),
    (r'\b2759128\b', '旧 baseline 2759128'),
    (r'ros2\s+(launch|run|param)', 'ros2 命令（项目是 ROS1 Noetic）'),
    (r'ros__parameters', 'ros__parameters（ROS2 遗留）'),
    (r'alarm_code\s*=\s*0', 'alarm_code=0 作为当前 clear（旧协议）'),
    (r'当前现场验证基线[^\n]*8d7d7ee', '8d7d7ee 写成当前基线'),
]


def check_stale_current_content():
    """检查 current docs 是否含过期事实（排除 archive 与 validation 历史证据）。"""
    current_targets = [
        os.path.join(REPO_ROOT, 'README.md'),
        os.path.join(REPO_ROOT, 'CHANGELOG.md'),
        os.path.join(REPO_ROOT, 'SAFETY.md'),
        os.path.join(REPO_ROOT, 'docs', 'project'),
        os.path.join(REPO_ROOT, 'docs', 'design'),
        os.path.join(REPO_ROOT, 'src', 'ndt_slam', 'doc'),
    ]
    errors = []
    for target in current_targets:
        paths = []
        if os.path.isfile(target):
            paths.append(target)
        elif os.path.isdir(target):
            for dp, _, fns in os.walk(target):
                for fn in fns:
                    if fn.endswith('.md'):
                        paths.append(os.path.join(dp, fn))
        for p in paths:
            with open(p, 'r', encoding='utf-8', errors='replace') as f:
                for lineno, line in enumerate(f, 1):
                    for pat, desc in STALE_CURRENT_PATTERNS:
                        if re.search(pat, line):
                            errors.append(
                                f"{p}:{lineno}: 过期事实 '{desc}': "
                                f"{line.strip()[:60]}")
    return errors


def check_baseline_consistency():
    """校验 README / status 的 baseline 与 current_baseline.yaml 一致。"""
    baseline_yaml = os.path.join(REPO_ROOT, 'docs', 'project',
                                 'current_baseline.yaml')
    if not os.path.exists(baseline_yaml):
        return ["docs/project/current_baseline.yaml 不存在"]
    import yaml
    with open(baseline_yaml, 'r', encoding='utf-8') as f:
        baseline = yaml.safe_load(f)
    tag = baseline.get('field_baseline', {}).get('tag', '')
    sha = baseline.get('field_baseline', {}).get('sha', '')
    errors = []
    for doc_name, path in [
        ('README.md', os.path.join(REPO_ROOT, 'README.md')),
        ('status.md', os.path.join(REPO_ROOT, 'docs', 'project', 'status.md')),
    ]:
        if not os.path.exists(path):
            errors.append(f"{doc_name} 不存在")
            continue
        with open(path, 'r', encoding='utf-8', errors='replace') as f:
            content = f.read()
        if tag and tag not in content:
            errors.append(f"{doc_name} 缺少 baseline tag '{tag}'")
        if sha and sha not in content:
            errors.append(f"{doc_name} 缺少 baseline sha '{sha}'")
    return errors


def check_no_historical_dirs():
    """历史目录（archive/validation/design）不允许在当前树重新出现。"""
    forbidden_dirs = [
        os.path.join(REPO_ROOT, 'docs', 'archive'),
        os.path.join(REPO_ROOT, 'docs', 'validation'),
        os.path.join(REPO_ROOT, 'docs', 'design'),
    ]
    errors = []
    for d in forbidden_dirs:
        if os.path.isdir(d):
            errors.append(f"{d} 历史目录不应在当前树重新出现（由 Git 历史承担）")
    return errors


def main():
    all_errors = []

    # 1. Markdown 链接检查
    md_files = find_md_files(REPO_ROOT)
    all_errors.extend(check_markdown_links(md_files))

    # 2. README 禁止项
    readme_path = os.path.join(REPO_ROOT, 'README.md')
    all_errors.extend(check_readme_forbidden(readme_path))

    # 3. 技术文档 legacy 引用
    tech_doc_dir = os.path.join(REPO_ROOT, 'src', 'ndt_slam', 'doc')
    tech_docs = []
    if os.path.isdir(tech_doc_dir):
        tech_docs = [os.path.join(tech_doc_dir, f) for f in os.listdir(tech_doc_dir)
                     if f.endswith('.md')]
    all_errors.extend(check_legacy_references(tech_docs))

    # 4. Roadmap 位置
    all_errors.extend(check_roadmap_not_in_tech_docs())

    # 5. 技术文档中文标题检查
    tech_doc_dir = os.path.join(REPO_ROOT, 'src', 'ndt_slam', 'doc')
    all_errors.extend(check_canonical_docs_language(tech_doc_dir))

    # 6. Docs 索引链接
    docs_dir = os.path.join(REPO_ROOT, 'docs')
    if os.path.isdir(docs_dir):
        all_errors.extend(check_docs_index_links(docs_dir))
    if os.path.isdir(tech_doc_dir):
        all_errors.extend(check_docs_index_links(tech_doc_dir))

    # 7. 过期事实禁止（current docs）
    all_errors.extend(check_stale_current_content())

    # 8. baseline 一致性
    all_errors.extend(check_baseline_consistency())

    # 9. 历史目录不允许重新出现
    all_errors.extend(check_no_historical_dirs())

    if all_errors:
        print("文档合同检查发现以下问题：")
        for err in all_errors:
            print(f"  FAIL: {err}")
        print(f"\n共 {len(all_errors)} 个问题。")
        sys.exit(1)
    else:
        print("PASS: 文档合同检查通过")
        sys.exit(0)


if __name__ == '__main__':
    main()
