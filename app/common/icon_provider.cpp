#include "common/icon_provider.h"

#include <QFile>
#include <QGuiApplication>
#include <QPainter>
#include <QPixmap>
#include <QScreen>
#include <QSvgRenderer>

namespace wpiemu::app {

namespace {

QString resourcePath(const QString& name) { return QStringLiteral(":/wpiemu/icons/%1.svg").arg(name); }

QPixmap render(const QByteArray& svg, int size, qreal dpr) {
    QSvgRenderer renderer(svg);
    QPixmap pixmap(QSize(size, size) * dpr);
    pixmap.setDevicePixelRatio(dpr);
    pixmap.fill(Qt::transparent);
    if (renderer.isValid()) {
        QPainter painter(&pixmap);
        renderer.render(&painter, QRectF(0, 0, size, size));
    }
    return pixmap;
}

QIcon buildIcon(const QByteArray& svg) {
    QIcon icon;
    for (const int size : {16, 20, 24, 32, 48, 64})
        for (const qreal dpr : {1.0, 2.0}) icon.addPixmap(render(svg, size, dpr));
    return icon;
}

}  // namespace

QString IconProvider::coloredSvg(const QString& name, const QColor& color) {
    QFile file(resourcePath(name));
    if (!file.open(QIODevice::ReadOnly)) return {};
    QString svg = QString::fromUtf8(file.readAll());
    svg.replace(QLatin1String("currentColor"), color.name(QColor::HexRgb));
    return svg;
}

QIcon IconProvider::icon(const QString& name, const QColor& color) const {
    const QString key = name + QLatin1Char('|') + color.name(QColor::HexRgb);
    if (const auto it = cache_.constFind(key); it != cache_.constEnd()) return *it;
    const QString svg = coloredSvg(name, color);
    QIcon icon = svg.isEmpty() ? QIcon() : buildIcon(svg.toUtf8());
    cache_.insert(key, icon);
    return icon;
}

QIcon IconProvider::logo() const {
    QFile file(resourcePath(QStringLiteral("app")));
    if (!file.open(QIODevice::ReadOnly)) return {};
    return buildIcon(file.readAll());
}

}  // namespace wpiemu::app
