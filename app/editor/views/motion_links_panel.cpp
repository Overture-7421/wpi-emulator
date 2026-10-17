#include "views/motion_links_panel.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QVBoxLayout>

#include "model/tr.h"
#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/mechanisms/motion_config.h"

namespace wpiemu::app::editor {

MotionLinksPanel::MotionLinksPanel(EditorController& controller, QWidget* parent)
    : QWidget(parent) {
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    auto* help = new QLabel(
        ETR("A source drives its followers' motion and sensors. Negative ratios reverse motion. "
            "Followers need no motor and do not load the source."),
        this);
    help->setWordWrap(true);
    layout->addWidget(help);
    auto* list = new QListWidget(this);
    list->setObjectName(QStringLiteral("motionLinks.list"));
    list->setMaximumHeight(140);
    const auto config = controller.document().config();
    const auto axes = sdk::motionAxisInfos(config, [&](std::string_view type) {
        return controller.document().catalog().findTemplate(type);
    });
    for (const auto& link : config.motionLinks)
        list->addItem(
            QString::fromStdString(link.source.toString() + " -> " + link.target.toString()));
    layout->addWidget(list);
    auto* source = new QComboBox(this);
    auto* target = new QComboBox(this);
    source->setObjectName(QStringLiteral("motionLinks.source"));
    target->setObjectName(QStringLiteral("motionLinks.target"));
    for (const auto& axis : axes) {
        const auto label = QString::fromStdString(axis.address.toString());
        source->addItem(label);
        target->addItem(label);
    }
    if (axes.size() > 1) target->setCurrentIndex(1);
    auto* ratio = new QDoubleSpinBox(this);
    ratio->setObjectName(QStringLiteral("motionLinks.ratio"));
    ratio->setRange(-1e9, 1e9);
    ratio->setDecimals(9);
    ratio->setValue(1.0);
    auto* units = new QLabel(this);
    auto* form = new QFormLayout;
    form->addRow(ETR("Source"), source);
    form->addRow(ETR("Follower"), target);
    form->addRow(ETR("Ratio"), ratio);
    form->addRow(ETR("Units"), units);
    layout->addLayout(form);
    auto* error = new QLabel(this);
    error->setObjectName(QStringLiteral("motionLinks.error"));
    error->setTextFormat(Qt::PlainText);
    error->setWordWrap(true);
    layout->addWidget(error);
    const auto factor = [axes](int index) {
        return index >= 0 && axes[static_cast<std::size_t>(index)].kind == sdk::PlantKind::Angular
                   ? sdk::kTwoPi
                   : 1.0;
    };
    const auto updateUnits = [axes, source, target, units] {
        const auto unit = [&](int i) {
            return i >= 0 && axes[static_cast<std::size_t>(i)].kind == sdk::PlantKind::Angular
                       ? ETR("rev")
                       : ETR("m");
        };
        units->setText(unit(target->currentIndex()) + QStringLiteral(" / ") +
                       unit(source->currentIndex()));
    };
    connect(source, &QComboBox::currentIndexChanged, this, [updateUnits] { updateUnits(); });
    connect(target, &QComboBox::currentIndexChanged, this, [updateUnits] { updateUnits(); });
    updateUnits();
    connect(list, &QListWidget::currentRowChanged, this, [=](int row) {
        if (row < 0) return;
        const auto& link = config.motionLinks[static_cast<std::size_t>(row)];
        source->setCurrentIndex(-1);
        target->setCurrentIndex(-1);
        for (std::size_t i = 0; i < axes.size(); ++i) {
            if (axes[i].address == link.source) source->setCurrentIndex(static_cast<int>(i));
            if (axes[i].address == link.target) target->setCurrentIndex(static_cast<int>(i));
        }
        ratio->setValue(link.ratio * factor(source->currentIndex()) /
                        factor(target->currentIndex()));
    });
    auto* add = new QPushButton(ETR("Add / update link"), this);
    add->setObjectName(QStringLiteral("motionLinks.apply"));
    layout->addWidget(add);
    connect(add, &QPushButton::clicked, this, [=, &controller] {
        if (source->currentIndex() < 0 || target->currentIndex() < 0) {
            error->setText(ETR("Select existing source and follower axes."));
            return;
        }
        auto candidate = controller.document().config();
        sdk::MotionLink link;
        const int row = list->currentRow();
        if (row >= 0)
            link.id = config.motionLinks[static_cast<std::size_t>(row)].id;
        else {
            int n = 1;
            do {
                link.id = "link" + std::to_string(n++);
            } while (std::any_of(candidate.motionLinks.begin(), candidate.motionLinks.end(),
                                 [&](const auto& existing) { return existing.id == link.id; }));
        }
        link.source = axes[static_cast<std::size_t>(source->currentIndex())].address;
        link.target = axes[static_cast<std::size_t>(target->currentIndex())].address;
        link.ratio =
            ratio->value() * factor(target->currentIndex()) / factor(source->currentIndex());
        if (row >= 0)
            candidate.motionLinks[static_cast<std::size_t>(row)] = link;
        else
            candidate.motionLinks.push_back(link);
        const auto issues = sdk::motionLinkIssues(candidate, axes);
        if (!issues.empty()) {
            error->setText(QString::fromStdString(issues.front().message));
            return;
        }
        controller.document().edit(ETR("Edit motion link"), [candidate](sdk::RobotConfig& c) {
            c.motionLinks = candidate.motionLinks;
            c.motionLinksConfigured = true;
            return true;
        });
    });
    auto* fresh = new QPushButton(ETR("New link"), this);
    connect(fresh, &QPushButton::clicked, this, [list] { list->setCurrentRow(-1); });
    layout->addWidget(fresh);
    auto* remove = new QPushButton(ETR("Remove selected link"), this);
    remove->setObjectName(QStringLiteral("motionLinks.remove"));
    layout->addWidget(remove);
    connect(remove, &QPushButton::clicked, this, [=, &controller] {
        const int row = list->currentRow();
        if (row < 0) return;
        const auto id = config.motionLinks[static_cast<std::size_t>(row)].id;
        controller.document().edit(ETR("Remove motion link"), [id](sdk::RobotConfig& c) {
            std::erase_if(c.motionLinks, [&](const auto& link) { return link.id == id; });
            c.motionLinksConfigured = true;
            return true;
        });
    });
    if (config.drivetrain.spec.kind == sdk::drivetrain_kind::kTank) {
        auto* preset = new QPushButton(ETR("Link tank wheels by side"), this);
        preset->setObjectName(QStringLiteral("motionLinks.tankPreset"));
        layout->addWidget(preset);
        connect(preset, &QPushButton::clicked, this, [error, &controller] {
            auto links = sdk::tankMotionLinks(controller.document().config());
            if (!links) {
                error->setText(QString::fromStdString(links.error().message));
                return;
            }
            controller.document().edit(ETR("Link tank wheels"), [&](sdk::RobotConfig& c) {
                std::erase_if(c.motionLinks, [](const auto& link) {
                    return link.target.owner == sdk::slot_owner::kDrivetrain;
                });
                for (auto link : links.value()) {
                    while (
                        std::any_of(c.motionLinks.begin(), c.motionLinks.end(),
                                    [&](const auto& existing) { return existing.id == link.id; }))
                        link.id += "_2";
                    c.motionLinks.push_back(std::move(link));
                }
                c.motionLinksConfigured = true;
                return true;
            });
        });
        auto* independent = new QPushButton(ETR("Use independent wheels"), this);
        layout->addWidget(independent);
        connect(independent, &QPushButton::clicked, this, [&controller] {
            controller.document().edit(ETR("Use independent wheels"), [](sdk::RobotConfig& c) {
                std::erase_if(c.motionLinks, [](const auto& link) {
                    return link.target.owner == sdk::slot_owner::kDrivetrain;
                });
                c.motionLinksConfigured = true;
                return true;
            });
        });
    }
}

}  // namespace wpiemu::app::editor
