#include "shell/about_dialog.h"

#include <QClipboard>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSysInfo>
#include <QVBoxLayout>

#include "common/ui_helpers.h"
#include "wpiemu/sdk/version.h"

namespace wpiemu::app {

QString aboutDetails(const sdk::GamePluginRegistry& plugins) {
    QStringList lines;
    lines << QStringLiteral("WPI Emulator %1").arg(QLatin1String(WPIEMU_APP_VERSION));
    lines << QStringLiteral("SDK %1.%2").arg(sdk::kSdkVersionMajor).arg(sdk::kSdkVersionMinor);
    lines << QStringLiteral("Qt %1").arg(QLatin1String(qVersion()));
    lines << QStringLiteral("OS %1").arg(QSysInfo::prettyProductName());
    lines << QStringLiteral("CPU %1").arg(QSysInfo::currentCpuArchitecture());
    QStringList games;
    for (const sdk::IGameModule* module : plugins.list()) {
        const sdk::GameMetadata meta = module->metadata();
        games << QStringLiteral("%1 (%2, v%3)").arg(QString::fromStdString(meta.displayName), QString::fromStdString(meta.id)).arg(meta.pluginVersion);
    }
    lines << QStringLiteral("Game modules: %1").arg(games.isEmpty() ? QStringLiteral("none") : games.join(QStringLiteral("; ")));
    return lines.join(QLatin1Char('\n'));
}

AboutDialog::AboutDialog(const sdk::GamePluginRegistry& plugins, const UiResources& ui, QWidget* parent) : QDialog(parent) {
    setWindowTitle(tr("About WPI Emulator"));
    setModal(true);
    setMinimumWidth(460);

    auto* logo = new QLabel;
    logo->setPixmap(ui.logo().pixmap(64, 64));
    auto* title = makeLabel(tr("WPI Emulator"), "title");
    auto* version = makeLabel(tr("Version %1").arg(QLatin1String(WPIEMU_APP_VERSION)), "subtitle");
    auto* tagline = makeLabel(tr("Runs unmodified WPILib robot projects (Java or C++) in a physics simulation."), "muted", nullptr, true);

    auto* head = new QHBoxLayout;
    head->setSpacing(16);
    head->addWidget(logo, 0, Qt::AlignTop);
    auto* headText = new QVBoxLayout;
    headText->addWidget(title);
    headText->addWidget(version);
    head->addLayout(headText, 1);

    auto* details = new QLabel(aboutDetails(plugins));
    details->setTextInteractionFlags(Qt::TextSelectableByMouse);
    details->setProperty("role", QStringLiteral("muted"));
    auto* detailsCard = makeCard();
    auto* detailsLayout = new QVBoxLayout(detailsCard);
    detailsLayout->setContentsMargins(14, 12, 14, 12);
    detailsLayout->addWidget(details);

    auto* copy = new QPushButton(tr("Copy details"));
    auto* close = new QPushButton(tr("Close"));
    close->setProperty("primary", true);
    close->setDefault(true);
    auto* buttons = new QHBoxLayout;
    buttons->addWidget(copy);
    buttons->addStretch(1);
    buttons->addWidget(close);

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(24, 24, 24, 18);
    root->setSpacing(14);
    root->addLayout(head);
    root->addWidget(tagline);
    root->addWidget(detailsCard);
    root->addLayout(buttons);

    const QString text = details->text();
    connect(copy, &QPushButton::clicked, this, [text] { QGuiApplication::clipboard()->setText(text); });
    connect(close, &QPushButton::clicked, this, &QDialog::accept);
}

}  // namespace wpiemu::app
