#!/usr/bin/env python3
"""检查文档合同：链接有效性、README 禁止项、过期引用、索引一致性。"""

import os
import re
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


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
    """检查 README 不应包含的内容。"""
    if not os.path.exists(readme_path):
        return ["README.md 不存在"]
    with open(readme_path, 'r', encoding='utf-8', errors='replace') as f:
        content = f.read()
    forbidden = [
        'Known Limitations',
        'Not yet',
        'TODO',
        'NOT_RUN',
    ]
    errors = []
    for term in forbidden:
        if term in content:
            errors.append(f"README.md 包含禁止项: '{term}'")
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

    # 5. Docs 索引链接
    docs_dir = os.path.join(REPO_ROOT, 'docs')
    if os.path.isdir(docs_dir):
        all_errors.extend(check_docs_index_links(docs_dir))
    if os.path.isdir(tech_doc_dir):
        all_errors.extend(check_docs_index_links(tech_doc_dir))

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
